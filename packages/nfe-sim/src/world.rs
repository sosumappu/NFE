use rstar::{PointDistance, RTree, RTreeObject, AABB};
use serde::Deserialize;
/// sim/world.rs — Static world representation (walls as line segments)
///
/// Loaded from a JSON file produced by your coworker's Python map tool:
///
///   {
///     "inner_walls":     [[x1,y1], ...],
///     "outer_walls":     [[x1,y1], ...],
///     "start":     {"x": 0.0, "y": 0.0, "yaw_rad": 0.0},
///     "waypoints": [[x,y], ...]   // optional, for lap timing
///   }
///
/// All coordinates in metres, world frame (+x east, +y north is fine —
/// the simulator transforms into car-local frame per tick).
use std::{fs, path::Path};

// ── Geometry ───────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy)]
pub struct Seg {
    pub ax: f32,
    pub ay: f32,
    pub bx: f32,
    pub by: f32,
}

impl Seg {
    /// Ray from (ox,oy) in direction (dx,dy).
    /// Returns t ≥ 0 (distance along ray) or f32::MAX if no intersection.
    #[inline]
    pub fn ray_intersect(&self, ox: f32, oy: f32, dx: f32, dy: f32) -> f32 {
        let ex = self.bx - self.ax;
        let ey = self.by - self.ay;
        let denom = dx * ey - dy * ex;
        if denom.abs() < 1e-9 {
            return f32::MAX;
        }
        let fx = self.ax - ox;
        let fy = self.ay - oy;
        let t = (fx * ey - fy * ex) / denom;
        let u = (fx * dy - fy * dx) / denom;
        if t >= 0.0 && (0.0..=1.0).contains(&u) {
            t
        } else {
            f32::MAX
        }
    }
}

// ── World ──────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct StartPose {
    pub x: f32,
    pub y: f32,
    pub yaw_rad: f32,
}

#[derive(Debug, Clone)]
pub struct World {
    pub inner_walls: RTree<Seg>,
    pub outer_walls: RTree<Seg>,
    pub start: StartPose,
    /// Lap waypoints — (x, y) in world frame, used for progress / cost
    pub waypoints: Vec<(f32, f32)>,
}

// ── JSON serde shims ───────────────────────────────────────────────────────

#[derive(Deserialize)]
struct JsonWorld {
    inner_walls: Vec<[f32; 2]>,
    outer_walls: Vec<[f32; 2]>,
    start: JsonStart,
    #[serde(default)]
    waypoints: Vec<[f32; 2]>,
}

#[derive(Deserialize)]
struct JsonStart {
    x: f32,
    y: f32,
    yaw_rad: f32,
}

#[inline]
fn loop_to_segs(points: &[[f32; 2]]) -> Vec<Seg> {
    let n = points.len();
    (0..n)
        .map(|i| {
            let a = points[i];
            let b = points[(i + 1) % n];
            Seg {
                ax: a[0],
                ay: a[1],
                bx: b[0],
                by: b[1],
            }
        })
        .collect()
}

impl RTreeObject for Seg {
    type Envelope = AABB<[f32; 2]>;

    fn envelope(&self) -> Self::Envelope {
        let min_x = self.ax.min(self.bx);
        let min_y = self.ay.min(self.by);
        let max_x = self.ax.max(self.bx);
        let max_y = self.ay.max(self.by);

        AABB::from_corners([min_x, min_y], [max_x, max_y])
    }
}

impl PointDistance for Seg {
    // Note: rstar expects the *squared* distance for optimization purposes,
    // so we don't take the square root here!
    fn distance_2(&self, point: &[f32; 2]) -> f32 {
        let px = point[0];
        let py = point[1];

        // Segment vector AB
        let abx = self.bx - self.ax;
        let aby = self.by - self.ay;

        // Vector from A to Point
        let apx = px - self.ax;
        let apy = py - self.ay;

        // Squared length of the segment
        let ab_len_sq = abx * abx + aby * aby;

        if ab_len_sq == 0.0 {
            // The segment is actually just a single point
            return apx * apx + apy * apy;
        }

        // Project point onto the line segment.
        // `t` is the distance along the segment, normalized to [0.0, 1.0].
        let t = (apx * abx + apy * aby) / ab_len_sq;
        let t_clamped = t.clamp(0.0, 1.0);

        // Find the closest point on the segment
        let cx = self.ax + t_clamped * abx;
        let cy = self.ay + t_clamped * aby;

        // Return the squared distance from the point to the closest point on the line
        let dx = px - cx;
        let dy = py - cy;

        dx * dx + dy * dy
    }
}

impl World {
    pub fn inner_segments(&self) -> impl Iterator<Item = Seg> + '_ {
        self.inner_walls.iter().copied()
    }

    pub fn outer_segments(&self) -> impl Iterator<Item = Seg> + '_ {
        self.outer_walls.iter().copied()
    }

    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let raw = fs::read_to_string(&path)?;
        let jw: JsonWorld = serde_json::from_str(&raw)?;

        let inner_segs = loop_to_segs(&jw.inner_walls);
        let outer_segs = loop_to_segs(&jw.outer_walls);
        Ok(Self {
            inner_walls: RTree::bulk_load(inner_segs),
            outer_walls: RTree::bulk_load(outer_segs),
            start: StartPose {
                x: jw.start.x,
                y: jw.start.y,
                yaw_rad: jw.start.yaw_rad,
            },
            waypoints: jw.waypoints.iter().map(|p| (p[0], p[1])).collect(),
        })
    }

    /// Cast a single ray from world position (wx, wy) at world-frame angle_rad.
    /// Returns distance to nearest wall, capped at max_dist.
    #[inline]
    pub fn raycast(&self, wx: f32, wy: f32, angle_rad: f32, max_dist: f32) -> f32 {
        let (s, c) = angle_rad.sin_cos();

        // Create a bounding box for the ray's maximum reach
        let end_x = wx + c * max_dist;
        let end_y = wy + s * max_dist;

        let min_x = wx.min(end_x);
        let min_y = wy.min(end_y);
        let max_x = wx.max(end_x);
        let max_y = wy.max(end_y);

        let ray_aabb = rstar::AABB::from_corners([min_x, min_y], [max_x, max_y]);

        // Only test intersections with walls inside the ray's bounding box
        let mut closest = max_dist;

        for seg in self.inner_walls.locate_in_envelope_intersecting(ray_aabb) {
            closest = closest.min(seg.ray_intersect(wx, wy, c, s));
        }
        for seg in self.outer_walls.locate_in_envelope_intersecting(ray_aabb) {
            closest = closest.min(seg.ray_intersect(wx, wy, c, s));
        }

        closest
    }

    /// Index of the next waypoint ahead of `pos`, or None if list is empty.
    pub fn next_waypoint(&self, _pos: (f32, f32), last_reached: usize) -> Option<usize> {
        if self.waypoints.is_empty() {
            return None;
        }
        Some((last_reached + 1) % self.waypoints.len())
    }

    /// Crash detection
    pub fn distance_to_closest_wall(&self, x: f32, y: f32) -> f32 {
        let point = [x, y];

        // rstar has built-in nearest neighbor calculations that are lightning fast
        let inner_dist = self
            .inner_walls
            .nearest_neighbor(point)
            .map(|seg| seg.distance_2(&point).sqrt()) // Assuming you implement distance logic
            .unwrap_or(f32::MAX);

        let outer_dist = self
            .outer_walls
            .nearest_neighbor(point)
            .map(|seg| seg.distance_2(&point).sqrt())
            .unwrap_or(f32::MAX);

        inner_dist.min(outer_dist)
    }
}
