use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};

use nfe_algo::mapping::{MapperParams, OccupancyGridMapper};
use nfe_core::mapping::{MapStatus, MapperClient, MappingInput, TrackMap};

#[derive(Debug)]
enum Work {
    Integrate(MappingInput),
    LapComplete,
    Shutdown,
}

pub struct MappingWorker {
    enabled: bool,
    tx: Option<mpsc::SyncSender<Work>>,
    status: Arc<Mutex<MapStatus>>,
    map: Arc<Mutex<Option<TrackMap>>>,
    handle: Option<JoinHandle<()>>,
}

impl MappingWorker {
    pub fn disabled() -> Self {
        Self {
            enabled: false,
            tx: None,
            status: Arc::new(Mutex::new(MapStatus {
                enabled: false,
                ..Default::default()
            })),
            map: Arc::new(Mutex::new(None)),
            handle: None,
        }
    }

    pub fn start(params: MapperParams, queue_capacity: usize, _seed: u64) -> Self {
        let (tx, rx) = mpsc::sync_channel(queue_capacity.max(1));
        let status = Arc::new(Mutex::new(MapStatus {
            enabled: true,
            ..Default::default()
        }));
        let map = Arc::new(Mutex::new(None));
        let status_thread = status.clone();
        let map_thread = map.clone();
        let handle = thread::Builder::new()
            .name("nfe-mapper".into())
            .spawn(move || {
                let mut mapper = OccupancyGridMapper::new(params);
                while let Ok(work) = rx.recv() {
                    match work {
                        Work::Integrate(input) => mapper.integrate(input),
                        Work::LapComplete => mapper.mark_lap_complete(),
                        Work::Shutdown => break,
                    }
                    let status_snapshot = mapper.status();
                    let map_snapshot = mapper.map();
                    {
                        let mut status = status_thread.lock().unwrap();
                        let dropped_scans = status.dropped_scans;
                        *status = status_snapshot;
                        status.dropped_scans = dropped_scans;
                    }
                    *map_thread.lock().unwrap() = Some(map_snapshot);
                }
            })
            .expect("spawn nfe-mapper");

        Self {
            enabled: true,
            tx: Some(tx),
            status,
            map,
            handle: Some(handle),
        }
    }

    pub fn mark_lap_complete(&mut self) {
        if let Some(tx) = &self.tx {
            let _ = tx.try_send(Work::LapComplete);
        }
    }

    pub fn shutdown(&mut self) {
        if let Some(tx) = self.tx.take() {
            let _ = tx.try_send(Work::Shutdown);
        }
        if let Some(handle) = self.handle.take() {
            let _ = handle.join();
        }
    }
}

impl Drop for MappingWorker {
    fn drop(&mut self) {
        self.shutdown();
    }
}

impl MapperClient for MappingWorker {
    fn submit(&mut self, input: MappingInput) -> bool {
        if !self.enabled {
            return false;
        }
        let Some(tx) = &self.tx else {
            return false;
        };
        match tx.try_send(Work::Integrate(input)) {
            Ok(()) => true,
            Err(mpsc::TrySendError::Full(_)) => {
                if let Ok(mut s) = self.status.lock() {
                    s.dropped_scans = s.dropped_scans.saturating_add(1);
                }
                false
            }
            Err(mpsc::TrySendError::Disconnected(_)) => false,
        }
    }

    fn latest_status(&self) -> MapStatus {
        *self.status.lock().unwrap()
    }

    fn latest_map(&self) -> Option<TrackMap> {
        self.map.try_lock().ok().and_then(|map| map.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::sensors::{LidarCloud, LidarPoint};
    use nfe_core::Pose2;

    fn scan(ts: u64) -> LidarCloud {
        let points = (0..24)
            .map(|i| {
                let angle = -0.6 + i as f32 * 0.05;
                LidarPoint::from_polar(1.0 + (i % 3) as f32 * 0.1, angle, ts)
            })
            .collect();
        LidarCloud {
            points,
            timestamp_us: ts,
        }
    }

    #[test]
    fn worker_backpressure_never_blocks_and_keeps_latest_available_map() {
        let params = MapperParams {
            resolution_m: 0.03,
            width_m: 8.0,
            height_m: 8.0,
            origin_x_m: -4.0,
            origin_y_m: -4.0,
            ..Default::default()
        };
        let mut worker = MappingWorker::start(params, 1, 0);
        let started = std::time::Instant::now();
        let attempts = 200u64;
        let mut accepted = 0u64;

        for i in 0..attempts {
            if worker.submit(MappingInput {
                cloud: scan(i + 1),
                pose: Pose2::new(i as f32 * 0.001, 0.0, 0.0),
                timestamp_us: i + 1,
            }) {
                accepted += 1;
            }
        }

        assert!(
            started.elapsed() < std::time::Duration::from_millis(250),
            "submit loop blocked for {:?}",
            started.elapsed()
        );
        assert!(accepted < attempts, "backpressure should drop work");

        let status = wait_for_processed(&worker, 1);
        assert!(status.processed_scans >= 1, "status={status:?}");
        assert!(status.dropped_scans > 0, "status={status:?}");

        let latest_started = std::time::Instant::now();
        let latest = worker.latest_map();
        assert!(
            latest_started.elapsed() < std::time::Duration::from_millis(50),
            "latest_map blocked for {:?}",
            latest_started.elapsed()
        );
        let latest = latest.expect("worker should publish latest available map");
        assert_eq!(latest.revision, worker.latest_status().latest_revision);

        worker.shutdown();
    }

    fn wait_for_processed(worker: &MappingWorker, processed: u64) -> MapStatus {
        for _ in 0..100 {
            let status = worker.latest_status();
            if status.processed_scans >= processed {
                return status;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        worker.latest_status()
    }
}
