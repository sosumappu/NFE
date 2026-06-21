use std::sync::{mpsc, Arc, Mutex};
use std::thread::{self, JoinHandle};

use nfe_algo::mapping::{MapperParams, RansacWallMapper};
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

    pub fn start(params: MapperParams, queue_capacity: usize, seed: u64) -> Self {
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
                let mut mapper = RansacWallMapper::new(params, seed);
                while let Ok(work) = rx.recv() {
                    match work {
                        Work::Integrate(input) => mapper.integrate(input),
                        Work::LapComplete => mapper.mark_lap_complete(),
                        Work::Shutdown => break,
                    }
                    *status_thread.lock().unwrap() = mapper.status();
                    *map_thread.lock().unwrap() = Some(mapper.map());
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
        if let Some(tx) = &self.tx {
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
        self.map.lock().unwrap().clone()
    }
}
