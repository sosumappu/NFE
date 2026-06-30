# Refactor architecture

This repo now has split-ready pure crates for the next autonomous-racing architecture:

- `packages/nfe-core`: shared geometry, sensor snapshots, control/estimation/mapping/localization/raceline trait boundaries, and the `Tunable` parameter registry.
- `packages/nfe-tunable-derive`: `#[derive(Tunable)]` proc macro for named, bounded parameter reflection.
- `packages/nfe-algo`: pure deterministic algorithms: RANSAC wall fitting, RANSAC corridor perception, EKF with IMU bias states, scan-match localization, particle fallback, explicit wall mapping, raceline generation/tracking, and supervisor health gates.
- `packages/nfe-runtime`: deterministic `Pipeline::step`, bounded mapping worker, start-line debouncer, sync run-loop traits, runtime config, and runtime replay helpers.
- `packages/nfe-tuner`: optimizer-independent search-space JSON, flat candidate application, candidate score JSON, and sim lap evaluation over the runtime pipeline.

The `packages/nfe-car` crate is the thin wiring layer for binaries and hardware adapters. Its control-loop wrapper delegates control decisions to `nfe-runtime::pipeline::Pipeline::step`, while replay/sim golden tests guard the reactive fallback behavior.

Key policy decisions encoded in the new modules:

- Reactive fallback is always available and can use `DeadReckonEstimator` without paying mapping/EKF overhead.
- Mapping is disabled by a single runtime flag: `RuntimeConfig.mapping.enabled`.
- Mapping runs asynchronously through `MapperClient`/`MappingWorker`; full queues drop scans instead of blocking the 100 Hz control loop.
- Lap completion is triggered by the physical start/finish line signal. Geometric loop closure is only a map-quality health check.
- The supervisor engages raceline mode only after map/raceline readiness plus localization/loop-closure hysteresis; sustained low confidence or estimator divergence falls back to reactive mode.
- Tuning search spaces are generated from `Tunable` descriptors; `nfe-tuner` evaluates sim candidates through the same pipeline used by runtime.
