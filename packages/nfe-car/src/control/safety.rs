use crate::config::SafetyConfig;

/// Returns (threat_present, estop_len)
pub fn estop_threat_cloud(
    cloud: &crate::types::LidarCloudView,
    v_current: f32,
    last_steering: f32,
    safety: &SafetyConfig,
) -> (bool, f32) {
    let base_len = (v_current * safety.t_lookahead_s).clamp(safety.estop_min_m, safety.estop_max_m);
    let tan_d = last_steering.tan().abs().max(safety.tan_min);
    let arc_cap = safety.c_arc * safety.wheelbase_m / tan_d;
    let estop_len = base_len.min(arc_cap);
    let curv = last_steering.tan() / (2.0 * safety.wheelbase_m);
    let threat = cloud.points.iter().any(|p| {
        p.x > 0.0 && p.x <= estop_len && (p.y - curv * p.x * p.x).abs() <= safety.half_channel_w_m
    });
    (threat, estop_len)
}

// ── Blind state machine ────────────────────────────────────────────
/// Pure state machine for the blind-during-fast-motion logic.
/// Tracks accumulated blind time; returns the action to take.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BlindAction {
    Normal,
    Coast { last_steering: f32 },
    SafeState,
}

#[derive(Debug, Clone, Copy)]
pub struct BlindState {
    pub blind_ms: u64,
}

impl Default for BlindState {
    fn default() -> Self {
        Self::new()
    }
}

impl BlindState {
    pub fn new() -> Self {
        Self { blind_ms: 0 }
    }

    pub fn update(
        &mut self,
        blind_now: bool,
        tick_dt_ms: u64,
        grace_ms: u64,
        last_steering: f32,
    ) -> BlindAction {
        if blind_now {
            self.blind_ms = self.blind_ms.saturating_add(tick_dt_ms);
        } else {
            self.blind_ms = 0;
        }

        if self.blind_ms == 0 {
            BlindAction::Normal
        } else if self.blind_ms < grace_ms {
            BlindAction::Coast { last_steering }
        } else {
            BlindAction::SafeState
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::SafetyConfig;
    use crate::types::{LidarCloudView, LidarPoint};

    fn make_point(x: f32, y: f32) -> LidarPoint {
        let dist = (x * x + y * y).sqrt();
        let angle = y.atan2(x);
        LidarPoint {
            x,
            y,
            dist_m: dist,
            angle_rad: angle,
            timestamp_us: 0,
        }
    }

    fn default_safety() -> SafetyConfig {
        SafetyConfig::default()
    }

    // ── Arc-cap clamping table from the design (Q3) ─────────────────
    // Steering δ  | tan δ  | arc_cap (m)  | base_len capped to?
    // 0°          | 0.0717 | 1.50         | ESTOP_MAX_M = 1.5 m
    // 5.7° (0.1r) | 0.100  | 1.08         | capped by arc_cap
    // 17° (0.3r)  | 0.309  | 0.348        | capped by arc_cap
    // 29° (0.5r)  | 0.546  | 0.197        | capped by arc_cap
    // 40° (0.7r)  | 0.842  | 0.128        | capped by arc_cap
    #[test]
    fn arc_cap_at_zero_steering_is_estop_max() {
        let safety = default_safety();
        let base_len = (5.0 * safety.t_lookahead_s).clamp(safety.estop_min_m, safety.estop_max_m);
        let tan_d = 0.0f32.tan().abs().max(safety.tan_min);
        let arc_cap = safety.c_arc * safety.wheelbase_m / tan_d;
        let estop_len = base_len.min(arc_cap);
        // high v→base_len clamped to ESTOP_MAX_M; arc_cap is huge (0.1075/0.0717≈1.5)
        assert!(
            (estop_len - safety.estop_max_m).abs() < 0.01,
            "zero steering: expected estop_max ({:.3}) got ({:.3})",
            safety.estop_max_m,
            estop_len
        );
    }

    #[test]
    fn arc_cap_at_full_lock() {
        let safety = default_safety();
        let steering = 0.7_f32; // ≈ 40°
        let tan_d = steering.tan().abs().max(safety.tan_min);
        let arc_cap = safety.c_arc * safety.wheelbase_m / tan_d;
        let expected = 0.5 * 0.215 / 0.7_f32.tan();
        assert!(
            (arc_cap - expected).abs() < 0.005,
            "full lock: expected {:.4}m, got {:.4}m",
            expected,
            arc_cap
        );
        // base_len at any speed is capped by this ~0.128 m
        let estop_len = (3.0_f32).min(arc_cap); // clamp irrelevant, arc_cap wins
        assert!(estop_len < safety.estop_min_m); // shorter than min at full lock
    }

    #[test]
    fn arc_cap_at_intermediate_steer() {
        let safety = default_safety();
        // 0.3 rad ≈ 17° → expected ≈ 0.348
        let steering = 0.3_f32;
        let tan_d = steering.tan().abs().max(safety.tan_min);
        let arc_cap = safety.c_arc * safety.wheelbase_m / tan_d;
        let expected = 0.5 * 0.215 / 0.3_f32.tan();
        assert!(
            (arc_cap - expected).abs() < 0.005,
            "17° steer: expected {:.4}m, got {:.4}m",
            expected,
            arc_cap
        );
    }

    #[test]
    fn point_inside_straight_channel_trips() {
        let safety = default_safety();
        // At v=0, estop_len = ESTOP_MIN_M = 0.25 m. Point must have x <= 0.25.
        let points = [make_point(0.20, 0.0)];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let (threat, estop_len) = estop_threat_cloud(&cloud, 0.0, 0.0, &safety);
        assert!(
            threat,
            "point at (0.20, 0) should trip; estop_len={:.3}",
            estop_len
        );
    }

    #[test]
    fn point_outside_channel_does_not_trip() {
        let safety = default_safety();
        let points = [make_point(0.5, 0.5)];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let (threat, _) = estop_threat_cloud(&cloud, 0.0, 0.0, &safety);
        assert!(
            !threat,
            "point at (0.5, 0.5) should not trip (y outside channel)"
        );
    }

    #[test]
    fn curvature_narrows_channel() {
        let safety = default_safety();
        // A point inside the straight channel but outside the curved one should
        // trip the straight-channel geometry check and not trip at nonzero steering.
        // Use (x=0.20, y=-0.12): |y| = 0.12 <= half_channel_w (≈0.13), so straight channel trips.
        let points = [make_point(0.20, -0.12)];

        let (threat_straight, _) = estop_threat_cloud(
            &LidarCloudView {
                points: &points,
                timestamp_us: 0,
            },
            0.0,
            0.0,
            &safety,
        );
        assert!(
            threat_straight,
            "point at (0.20,-0.12) should trip straight-channel check"
        );

        // With non-zero steering the curved centreline shifts and the same point
        // falls outside the curved channel.
        let (threat_curved, _) = estop_threat_cloud(
            &LidarCloudView {
                points: &points,
                timestamp_us: 0,
            },
            0.0,
            0.6,
            &safety,
        );
        assert!(
            !threat_curved,
            "point that trips straight-channel must not trip curved-channel at nonzero steer"
        );
    }

    #[test]
    fn ninety_degree_turn_no_false_trip() {
        // Synthetic unit test approximating the geometry encountered during a
        // tight 90° corner. We sweep a few representative steering values and
        // ensure inner-wall points that are close do not produce a false ESTOP.
        let safety = default_safety();

        // sample point near the inner wall at moderate forward distance
        // Choose x near estop_len (0.25) so curvature shifts can move the centreline
        // enough to exclude the point when steering is applied.
        let p = make_point(0.24, -0.12);

        // At zero steering the point should trip (|y|=0.12 <= 0.13 and x<=estop_len)
        let (threat0, estop_len0) = estop_threat_cloud(
            &LidarCloudView {
                points: &[p],
                timestamp_us: 0,
            },
            0.0,
            0.0,
            &safety,
        );
        assert!(
            threat0,
            "expected straight-channel threat at zero steering (estop_len={estop_len0:.3})"
        );

        // With non-zero steering the curvature centerline shifts enough that the
        // same point falls outside the curved channel and must NOT trip.
        for &steer in &[0.2_f32, 0.4_f32, 0.6_f32] {
            let (threat, estop_len) = estop_threat_cloud(
                &LidarCloudView {
                    points: &[p],
                    timestamp_us: 0,
                },
                0.0,
                steer,
                &safety,
            );
            assert!(
                !threat,
                "unexpected ESTOP at steer={steer} estop_len={estop_len:.3}"
            );
        }
    }

    #[test]
    fn point_behind_vehicle_ignored() {
        let safety = default_safety();
        let points = [make_point(-0.5, 0.0)];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let (threat, _) = estop_threat_cloud(&cloud, 0.0, 0.0, &safety);
        assert!(!threat, "point behind (x < 0) must not trip");
    }

    #[test]
    fn point_beyond_estop_len_ignored() {
        let safety = default_safety();
        // At v=0, base_len = ESTOP_MIN_M = 0.25
        let points = [make_point(10.0, 0.0)];
        let cloud = LidarCloudView {
            points: &points,
            timestamp_us: 0,
        };
        let (threat, estop_len) = estop_threat_cloud(&cloud, 0.0, 0.0, &safety);
        assert!(
            !threat,
            "point at x={:.1} > estop_len={:.3} must not trip",
            10.0, estop_len
        );
    }

    // ── Blind state machine tests ─────────────────────────────────
    #[test]
    fn blind_normal_when_not_blind() {
        let mut bs = BlindState::new();
        let action = bs.update(false, 10, 350, 0.5);
        assert_eq!(action, BlindAction::Normal);
        assert_eq!(bs.blind_ms, 0);
    }

    #[test]
    fn blind_coasts_during_grace_window() {
        let mut bs = BlindState::new();
        let action = bs.update(true, 10, 350, 0.5);
        assert_eq!(action, BlindAction::Coast { last_steering: 0.5 });
        assert_eq!(bs.blind_ms, 10);
    }

    #[test]
    fn blind_accumulates_across_ticks() {
        let mut bs = BlindState::new();
        bs.update(true, 10, 350, 0.0);
        bs.update(true, 10, 350, 0.0);
        bs.update(true, 10, 350, 0.0);
        assert_eq!(bs.blind_ms, 30);
    }

    #[test]
    fn blind_resets_on_good_tick() {
        let mut bs = BlindState::new();
        bs.update(true, 100, 350, 0.0); // blind, 100 ms
        bs.update(true, 100, 350, 0.0); // blind, 200 ms
        let action = bs.update(false, 10, 350, 0.0); // good!
        assert_eq!(action, BlindAction::Normal);
        assert_eq!(bs.blind_ms, 0);
    }

    #[test]
    fn blind_transitions_to_safe_state_at_grace_boundary() {
        let mut bs = BlindState::new();
        // 34 ticks of 10 ms each = 340 ms (just under grace)
        for _ in 0..34 {
            let action = bs.update(true, 10, 350, 0.0);
            assert_eq!(action, BlindAction::Coast { last_steering: 0.0 });
        }
        assert_eq!(bs.blind_ms, 340);
        // one more tick pushes it over
        let action = bs.update(true, 10, 350, 0.0);
        assert_eq!(action, BlindAction::SafeState);
        assert_eq!(bs.blind_ms, 350);
    }

    #[test]
    fn blind_single_good_tick_mid_blind_resets_counter() {
        let mut bs = BlindState::new();
        bs.update(true, 50, 350, 1.0);
        bs.update(true, 50, 350, 1.0);
        // single good tick resets to Normal
        let action = bs.update(false, 10, 350, 1.0);
        assert_eq!(action, BlindAction::Normal);
        // next blind starts fresh
        let action = bs.update(true, 10, 350, 1.0);
        assert_eq!(action, BlindAction::Coast { last_steering: 1.0 });
        assert_eq!(bs.blind_ms, 10);
    }
}
