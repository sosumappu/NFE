use std::sync::{mpsc, Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use nfe_algo::raceline::solver::{solve_min_curvature, RaceLineError, RaceLineSolverParams};
use nfe_core::mapping::TrackMap;
use nfe_core::raceline::RaceLine;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RaceLinePlannerSubmit {
    /// The map was accepted into the worker's single latest-map slot.
    Accepted,
    /// The same revision is already solving, pending, or completed.
    Duplicate,
    /// A newer revision replaced the one pending slot.
    ReplacedPending,
    /// The submitted revision is older than work already solving, pending, or done.
    BusyCurrentKept,
    Disabled,
}

#[derive(Clone, Debug)]
pub enum RaceLinePlannerEvent {
    Started {
        revision: u64,
    },
    Completed {
        revision: u64,
        line: RaceLine,
        solve_ms: u64,
    },
    Failed {
        revision: u64,
        error: RaceLineError,
        solve_ms: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RaceLinePlannerStatus {
    Disabled,
    Idle {
        latest_completed_revision: Option<u64>,
    },
    Pending {
        revision: u64,
    },
    Solving {
        revision: u64,
    },
    SolvingWithPending {
        solving_revision: u64,
        pending_revision: u64,
    },
}

type SolverFn = dyn Fn(&TrackMap, &RaceLineSolverParams) -> Result<RaceLine, RaceLineError>
    + Send
    + Sync
    + 'static;

#[derive(Default)]
struct SharedState {
    shutdown: bool,
    solving_revision: Option<u64>,
    pending: Option<TrackMap>,
    latest_completed_revision: Option<u64>,
}

pub struct RaceLinePlannerWorker {
    enabled: bool,
    state: Arc<(Mutex<SharedState>, Condvar)>,
    events: mpsc::Receiver<RaceLinePlannerEvent>,
    handle: Option<JoinHandle<()>>,
}

impl RaceLinePlannerWorker {
    pub fn disabled() -> Self {
        let (_tx, events) = mpsc::channel();
        Self {
            enabled: false,
            state: Arc::new((Mutex::new(SharedState::default()), Condvar::new())),
            events,
            handle: None,
        }
    }

    pub fn start(params: RaceLineSolverParams) -> Self {
        Self::start_with_solver(params, Arc::new(solve_min_curvature))
    }

    fn start_with_solver(params: RaceLineSolverParams, solver: Arc<SolverFn>) -> Self {
        let state = Arc::new((Mutex::new(SharedState::default()), Condvar::new()));
        let state_thread = state.clone();
        let (event_tx, events) = mpsc::channel();
        let handle = thread::Builder::new()
            .name("nfe-raceline-planner".into())
            .spawn(move || worker_loop(state_thread, event_tx, params, solver))
            .expect("spawn nfe-raceline-planner");

        Self {
            enabled: true,
            state,
            events,
            handle: Some(handle),
        }
    }

    /// Submit the latest complete map for planning.
    ///
    /// The worker keeps at most one actively solving revision and one pending
    /// revision. A newer submission replaces the pending slot; older submissions
    /// are discarded. This intentionally converges toward the latest map instead
    /// of building a backlog behind expensive QP solves.
    pub fn submit_latest(&self, map: TrackMap) -> RaceLinePlannerSubmit {
        if !self.enabled {
            return RaceLinePlannerSubmit::Disabled;
        }

        let (lock, cvar) = &*self.state;
        let mut state = lock.lock().unwrap();
        if state.shutdown {
            return RaceLinePlannerSubmit::Disabled;
        }

        let revision = map.revision;
        if state.solving_revision == Some(revision)
            || state
                .pending
                .as_ref()
                .is_some_and(|pending| pending.revision == revision)
            || state.latest_completed_revision == Some(revision)
        {
            return RaceLinePlannerSubmit::Duplicate;
        }
        if state
            .solving_revision
            .is_some_and(|solving| revision < solving)
            || state
                .pending
                .as_ref()
                .is_some_and(|pending| revision < pending.revision)
            || state
                .latest_completed_revision
                .is_some_and(|completed| revision < completed)
        {
            return RaceLinePlannerSubmit::BusyCurrentKept;
        }

        let result = if state.pending.is_some() {
            state.pending = Some(map);
            RaceLinePlannerSubmit::ReplacedPending
        } else {
            state.pending = Some(map);
            RaceLinePlannerSubmit::Accepted
        };
        cvar.notify_one();
        result
    }

    pub fn poll_event(&self) -> Option<RaceLinePlannerEvent> {
        self.events.try_recv().ok()
    }

    pub fn status(&self) -> RaceLinePlannerStatus {
        if !self.enabled {
            return RaceLinePlannerStatus::Disabled;
        }
        let (lock, _) = &*self.state;
        let state = lock.lock().unwrap();
        match (
            state.solving_revision,
            state.pending.as_ref().map(|pending| pending.revision),
        ) {
            (Some(solving_revision), Some(pending_revision)) => {
                RaceLinePlannerStatus::SolvingWithPending {
                    solving_revision,
                    pending_revision,
                }
            }
            (Some(revision), None) => RaceLinePlannerStatus::Solving { revision },
            (None, Some(revision)) => RaceLinePlannerStatus::Pending { revision },
            (None, None) => RaceLinePlannerStatus::Idle {
                latest_completed_revision: state.latest_completed_revision,
            },
        }
    }

    pub fn shutdown(&mut self) {
        if !self.enabled {
            return;
        }
        {
            let (lock, cvar) = &*self.state;
            let mut state = lock.lock().unwrap();
            state.shutdown = true;
            state.pending = None;
            cvar.notify_one();
        }
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
        self.enabled = false;
    }
}

impl Drop for RaceLinePlannerWorker {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn worker_loop(
    state: Arc<(Mutex<SharedState>, Condvar)>,
    events: mpsc::Sender<RaceLinePlannerEvent>,
    params: RaceLineSolverParams,
    solver: Arc<SolverFn>,
) {
    loop {
        let map = {
            let (lock, cvar) = &*state;
            let mut state = lock.lock().unwrap();
            while state.pending.is_none() && !state.shutdown {
                state = cvar.wait(state).unwrap();
            }
            if state.shutdown {
                break;
            }
            let map = state.pending.take().expect("pending checked above");
            state.solving_revision = Some(map.revision);
            map
        };

        let revision = map.revision;
        let _ = events.send(RaceLinePlannerEvent::Started { revision });
        let started = Instant::now();
        let result = solver(&map, &params);
        let solve_ms = started.elapsed().as_millis().min(u64::MAX as u128) as u64;

        match result {
            Ok(line) => {
                {
                    let (lock, cvar) = &*state;
                    let mut state = lock.lock().unwrap();
                    state.solving_revision = None;
                    state.latest_completed_revision = Some(revision);
                    cvar.notify_one();
                }
                let _ = events.send(RaceLinePlannerEvent::Completed {
                    revision,
                    line,
                    solve_ms,
                });
            }
            Err(error) => {
                {
                    let (lock, cvar) = &*state;
                    let mut state = lock.lock().unwrap();
                    state.solving_revision = None;
                    cvar.notify_one();
                }
                let _ = events.send(RaceLinePlannerEvent::Failed {
                    revision,
                    error,
                    solve_ms,
                });
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::raceline::RaceLinePoint;
    use nfe_core::Point2;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::time::Duration;

    fn map(revision: u64) -> TrackMap {
        TrackMap {
            revision,
            complete: true,
            ..Default::default()
        }
    }

    fn line(revision: u64) -> RaceLine {
        RaceLine {
            points: vec![RaceLinePoint {
                p: Point2::new(0.0, 0.0),
                speed_ms: 1.0,
                ..Default::default()
            }],
            closed: true,
            revision,
        }
    }

    #[test]
    fn submit_coalesces_to_latest_pending_revision() {
        let release = Arc::new(AtomicBool::new(false));
        let release_solver = release.clone();
        let solver = Arc::new(move |map: &TrackMap, _: &RaceLineSolverParams| {
            while !release_solver.load(Ordering::SeqCst) {
                std::thread::sleep(Duration::from_millis(1));
            }
            Ok(line(map.revision))
        });
        let mut worker =
            RaceLinePlannerWorker::start_with_solver(RaceLineSolverParams::default(), solver);

        assert_eq!(
            worker.submit_latest(map(1)),
            RaceLinePlannerSubmit::Accepted
        );
        assert!(matches!(
            wait_for_event(&worker),
            RaceLinePlannerEvent::Started { revision: 1 }
        ));
        assert_eq!(
            worker.submit_latest(map(1)),
            RaceLinePlannerSubmit::Duplicate
        );
        assert_eq!(
            worker.submit_latest(map(2)),
            RaceLinePlannerSubmit::Accepted
        );
        assert_eq!(
            worker.submit_latest(map(3)),
            RaceLinePlannerSubmit::ReplacedPending
        );
        assert_eq!(
            worker.submit_latest(map(2)),
            RaceLinePlannerSubmit::BusyCurrentKept
        );
        assert_eq!(
            worker.status(),
            RaceLinePlannerStatus::SolvingWithPending {
                solving_revision: 1,
                pending_revision: 3,
            }
        );

        release.store(true, Ordering::SeqCst);
        let mut started = Vec::new();
        let mut completed = Vec::new();
        for _ in 0..4 {
            match wait_for_event(&worker) {
                RaceLinePlannerEvent::Started { revision } => started.push(revision),
                RaceLinePlannerEvent::Completed { revision, .. } => completed.push(revision),
                RaceLinePlannerEvent::Failed {
                    revision, error, ..
                } => {
                    panic!("unexpected failure for {revision}: {error:?}")
                }
            }
            if completed == [1, 3] {
                break;
            }
        }

        assert!(started.contains(&3), "started={started:?}");
        assert_eq!(completed, vec![1, 3]);
        assert_eq!(
            worker.submit_latest(map(3)),
            RaceLinePlannerSubmit::Duplicate
        );
        assert_eq!(
            worker.submit_latest(map(2)),
            RaceLinePlannerSubmit::BusyCurrentKept
        );
        worker.shutdown();
    }

    fn wait_for_event(worker: &RaceLinePlannerWorker) -> RaceLinePlannerEvent {
        let started = Instant::now();
        loop {
            if let Some(event) = worker.poll_event() {
                return event;
            }
            assert!(
                started.elapsed() < Duration::from_secs(2),
                "timed out waiting for worker event; status={:?}",
                worker.status()
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }
}
