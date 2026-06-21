//! Runtime loop adapters around `Pipeline::step`.
//!
//! This module is intentionally minimal: live/replay/sim sources implement the
//! same traits, and tests/tuners can run `run_sync` deterministically.

use nfe_core::io::{ActuatorSink, SensorSource};

use crate::pipeline::{Pipeline, StepOutput};

pub fn run_sync(
    pipeline: &mut Pipeline,
    source: &mut dyn SensorSource,
    actuator: &mut dyn ActuatorSink,
    max_ticks: Option<usize>,
) -> anyhow::Result<Vec<StepOutput>> {
    let mut out = Vec::new();
    let mut ticks = 0usize;
    while max_ticks.is_none_or(|m| ticks < m) {
        let Some(snapshot) = source.next_snapshot()? else {
            break;
        };
        let step = pipeline.step(snapshot);
        actuator.apply(&step.command)?;
        out.push(step);
        ticks += 1;
    }
    actuator.safe_state()?;
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::RuntimeConfig;
    use crate::pipeline::{EstimatorMode, Pipeline};
    use nfe_core::estimation::ImuSample;
    use nfe_core::sensors::{LidarCloud, LidarPoint, SensorSnapshot};
    use nfe_core::Pose2;

    struct VecSource(Vec<SensorSnapshot>);
    impl SensorSource for VecSource {
        fn next_snapshot(&mut self) -> anyhow::Result<Option<SensorSnapshot>> {
            Ok(if self.0.is_empty() {
                None
            } else {
                Some(self.0.remove(0))
            })
        }
    }

    #[derive(Default)]
    struct Sink(usize);
    impl ActuatorSink for Sink {
        fn apply(&mut self, _output: &nfe_core::control::ControlOutput) -> anyhow::Result<()> {
            self.0 += 1;
            Ok(())
        }
        fn safe_state(&mut self) -> anyhow::Result<()> {
            Ok(())
        }
    }

    fn snap(ts: u64) -> SensorSnapshot {
        let mut cloud = LidarCloud {
            points: Vec::new(),
            timestamp_us: ts,
        };
        for i in 0..20 {
            let x = i as f32 * 0.05;
            cloud.points.push(LidarPoint {
                x,
                y: 0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: ts,
            });
            cloud.points.push(LidarPoint {
                x,
                y: -0.5,
                dist_m: x.hypot(0.5),
                angle_rad: 0.0,
                timestamp_us: ts,
            });
        }
        SensorSnapshot {
            lidar: cloud,
            imu: ImuSample {
                timestamp_us: ts,
                ..Default::default()
            },
            sensor_fault: false,
            sonar_m: [f32::MAX; 3],
            start_line_crossed: false,
        }
    }

    #[test]
    fn run_sync_drives_until_source_empty() {
        let mut cfg = RuntimeConfig::default();
        cfg.mapping.enabled = false;
        let mut p = Pipeline::new(cfg, EstimatorMode::DeadReckon);
        p.reset(Pose2::default(), 0);
        let mut src = VecSource(vec![snap(10_000), snap(20_000)]);
        let mut sink = Sink::default();
        let out = run_sync(&mut p, &mut src, &mut sink, None).unwrap();
        assert_eq!(out.len(), 2);
        assert_eq!(sink.0, 2);
    }
}
