//! RANSAC line fitting for corridor walls.
//!
//! Used by BOTH the reactive perception path and the mapping task — there is a
//! single wall-extraction implementation so the map is built from the same
//! geometry the controller reacts to. Input is car-local (or world, for the
//! mapper) points; output is fitted `WallLine`s in the same frame.
//!
//! Algorithm: sequential RANSAC. Repeatedly sample a 2-point hypothesis, count
//! inliers within `inlier_dist_m`, keep the best model, refine it by total
//! least squares over its inliers, then remove those inliers and repeat until
//! too few points remain or `max_walls` is reached. Deterministic given a seed
//! so replay/sim/tuning runs are reproducible.

use nfe_core::params::Tunable;
use nfe_core::{Point2, WallLine};

/// Tunable parameters for RANSAC wall fitting.
#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, Tunable)]
#[serde(default)]
pub struct RansacParams {
    /// Inlier threshold: max perpendicular distance to count as supporting.
    #[param(0.01..0.20, default = 0.05)]
    pub inlier_dist_m: f32,
    /// Minimum inliers for a hypothesis to be accepted as a wall.
    #[param(int, 4..40, default = 8)]
    pub min_inliers: usize,
    /// RANSAC iterations per wall extraction.
    #[param(int, 16..512, default = 80)]
    pub iterations: usize,
    /// Maximum walls to extract per scan (corridor: ~2-4).
    #[param(int, 1..8, default = 4)]
    pub max_walls: usize,
    /// Minimum endpoint separation for a valid hypothesis (reject near-coincident pairs).
    #[param(0.02..0.50, default = 0.08)]
    pub min_pair_sep_m: f32,
}

impl Default for RansacParams {
    fn default() -> Self {
        // Mirrors the derive defaults; kept explicit for non-tuned construction.
        Self {
            inlier_dist_m: 0.05,
            min_inliers: 8,
            iterations: 80,
            max_walls: 4,
            min_pair_sep_m: 0.08,
        }
    }
}

/// Small deterministic PRNG (xorshift64*). Avoids a rand dependency in the
/// pure-algo crate and guarantees reproducibility from a seed.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // Avoid the zero state, which is a fixed point of xorshift.
        Rng(seed | 1)
    }
    #[inline]
    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        self.0 = x;
        x.wrapping_mul(0x2545F4914F6CDD1D)
    }
    #[inline]
    fn below(&mut self, n: usize) -> usize {
        (self.next_u64() % (n as u64)) as usize
    }
}

/// Build a normalized `WallLine` from two distinct points; `None` if degenerate.
fn line_from_pair(a: &Point2, b: &Point2) -> Option<WallLine> {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    let len = dx.hypot(dy);
    if len < 1e-6 {
        return None;
    }
    // Unit normal is the direction rotated 90°.
    let nx = -dy / len;
    let ny = dx / len;
    let d = nx * a.x + ny * a.y;
    Some(WallLine {
        nx,
        ny,
        d,
        p0: *a,
        p1: *b,
        support: 0.0,
    })
}

/// Total-least-squares refinement of a line over its inlier set, with endpoints
/// re-derived as the extreme projections of the inliers along the line.
fn refit(points: &[Point2], inliers: &[usize]) -> Option<WallLine> {
    let n = inliers.len();
    if n < 2 {
        return None;
    }
    let inv = 1.0 / n as f32;
    let (mut mx, mut my) = (0.0f32, 0.0f32);
    for &i in inliers {
        mx += points[i].x;
        my += points[i].y;
    }
    mx *= inv;
    my *= inv;

    // Covariance of centered points.
    let (mut sxx, mut sxy, mut syy) = (0.0f32, 0.0f32, 0.0f32);
    for &i in inliers {
        let dx = points[i].x - mx;
        let dy = points[i].y - my;
        sxx += dx * dx;
        sxy += dx * dy;
        syy += dy * dy;
    }

    // Principal direction = eigenvector of the larger eigenvalue.
    let theta = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let (dir_s, dir_c) = theta.sin_cos();
    // Normal is perpendicular to the principal (line) direction.
    let nx = -dir_s;
    let ny = dir_c;
    let d = nx * mx + ny * my;

    // Endpoints: extreme projections onto the line direction.
    let mut t_min = f32::INFINITY;
    let mut t_max = f32::NEG_INFINITY;
    let (mut p0, mut p1) = (points[inliers[0]], points[inliers[0]]);
    for &i in inliers {
        let t = dir_c * (points[i].x - mx) + dir_s * (points[i].y - my);
        if t < t_min {
            t_min = t;
            p0 = points[i];
        }
        if t > t_max {
            t_max = t;
            p1 = points[i];
        }
    }

    Some(WallLine {
        nx,
        ny,
        d,
        p0,
        p1,
        support: 0.0,
    })
}

/// Extract corridor walls from a point set. `seed` makes the result
/// deterministic. Returns walls sorted by descending support.
pub fn fit_walls(points: &[Point2], params: &RansacParams, seed: u64) -> Vec<WallLine> {
    let mut walls = Vec::new();
    if points.len() < params.min_inliers {
        return walls;
    }

    let total = points.len() as f32;
    let mut rng = Rng::new(seed);
    // Indices still available for fitting; removed as walls claim them.
    let mut remaining: Vec<usize> = (0..points.len()).collect();
    let mut scratch_inliers: Vec<usize> = Vec::with_capacity(points.len());

    while walls.len() < params.max_walls && remaining.len() >= params.min_inliers {
        let mut best_inliers: Vec<usize> = Vec::new();

        for _ in 0..params.iterations {
            let ia = remaining[rng.below(remaining.len())];
            let ib = remaining[rng.below(remaining.len())];
            if ia == ib {
                continue;
            }
            let a = points[ia];
            let b = points[ib];
            if a.dist(&b) < params.min_pair_sep_m {
                continue;
            }
            let line = match line_from_pair(&a, &b) {
                Some(l) => l,
                None => continue,
            };

            scratch_inliers.clear();
            for &idx in &remaining {
                if line.signed_distance(&points[idx]).abs() <= params.inlier_dist_m {
                    scratch_inliers.push(idx);
                }
            }
            if scratch_inliers.len() > best_inliers.len() {
                best_inliers.clear();
                best_inliers.extend_from_slice(&scratch_inliers);
            }
        }

        if best_inliers.len() < params.min_inliers {
            break; // no more well-supported lines
        }

        if let Some(mut wall) = refit(points, &best_inliers) {
            wall.support = best_inliers.len() as f32 / total;
            walls.push(wall);
        }

        // Remove claimed inliers so the next iteration finds a different wall.
        let claimed: std::collections::HashSet<usize> = best_inliers.iter().copied().collect();
        remaining.retain(|i| !claimed.contains(i));
    }

    walls.sort_by(|a, b| {
        b.support
            .partial_cmp(&a.support)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    walls
}

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn line_points(n: usize, x0: f32, y0: f32, dx: f32, dy: f32, noise: f32) -> Vec<Point2> {
        // noise is deterministic (alternating sign) so the test is reproducible.
        (0..n)
            .map(|i| {
                let t = i as f32;
                let s = if i % 2 == 0 { 1.0 } else { -1.0 };
                Point2::new(x0 + dx * t + s * noise, y0 + dy * t - s * noise)
            })
            .collect()
    }

    #[test]
    fn fits_two_parallel_walls() {
        // Two corridor walls 1 m apart, both along +x.
        let mut pts = line_points(30, 0.0, 0.5, 0.1, 0.0, 0.01);
        pts.extend(line_points(30, 0.0, -0.5, 0.1, 0.0, 0.01));

        let walls = fit_walls(&pts, &RansacParams::default(), 42);
        assert!(walls.len() >= 2, "expected >=2 walls, got {}", walls.len());

        // Both walls should be roughly horizontal: normal ~ (0, ±1).
        for w in walls.iter().take(2) {
            assert!(
                w.nx.abs() < 0.2,
                "wall not horizontal: n=({},{})",
                w.nx,
                w.ny
            );
        }
    }

    #[test]
    fn deterministic_given_seed() {
        let pts = line_points(40, 0.0, 0.3, 0.08, 0.0, 0.02);
        let a = fit_walls(&pts, &RansacParams::default(), 7);
        let b = fit_walls(&pts, &RansacParams::default(), 7);
        assert_eq!(a.len(), b.len());
        for (x, y) in a.iter().zip(b.iter()) {
            assert_eq!(x.d, y.d);
            assert_eq!(x.nx, y.nx);
        }
    }

    #[test]
    fn too_few_points_returns_empty() {
        let pts = vec![Point2::new(0.0, 0.0), Point2::new(0.1, 0.0)];
        assert!(fit_walls(&pts, &RansacParams::default(), 1).is_empty());
    }

    #[test]
    fn refit_recovers_known_line() {
        // Points exactly on y = 0.5 line; refit normal should be (0, 1), d = 0.5.
        let pts: Vec<Point2> = (0..10).map(|i| Point2::new(i as f32 * 0.1, 0.5)).collect();
        let idx: Vec<usize> = (0..pts.len()).collect();
        let w = refit(&pts, &idx).unwrap();
        assert!(w.ny.abs() > 0.99, "normal should be vertical-ish: {:?}", w);
        assert!((w.d.abs() - 0.5).abs() < 1e-3, "offset wrong: {}", w.d);
    }

    proptest! {
        #[test]
        fn output_walls_are_normalized(seed in any::<u64>()) {
            let pts = line_points(30, 0.0, 0.4, 0.07, 0.01, 0.015);
            let walls = fit_walls(&pts, &RansacParams::default(), seed);
            for w in &walls {
                let norm = w.nx.hypot(w.ny);
                prop_assert!((norm - 1.0).abs() < 1e-3, "normal not unit: {}", norm);
                prop_assert!(w.support >= 0.0 && w.support <= 1.0);
            }
        }
    }
}
