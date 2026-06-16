use serde::Deserialize;
/// sim/world.rs — Static world representation (walls as line segments)
///
/// Loaded from a JSON file produced by your coworker's Python map tool:
///
///   {
///     "walls":     [[x1,y1,x2,y2], ...],
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
        if t >= 0.0 && u >= 0.0 && u <= 1.0 {
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
    pub walls: Vec<Seg>,
    pub start: StartPose,
    /// Lap waypoints — (x, y) in world frame, used for progress / cost
    pub waypoints: Vec<(f32, f32)>,
}

// ── JSON serde shims ───────────────────────────────────────────────────────

#[derive(Deserialize)]
struct JsonWorld {
    walls: Vec<[f32; 4]>,
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

impl World {
    pub fn load(path: impl AsRef<Path>) -> anyhow::Result<Self> {
        let raw = fs::read_to_string(&path)?;
        let jw: JsonWorld = serde_json::from_str(&raw)?;
        Ok(Self {
            walls: jw
                .walls
                .iter()
                .map(|w| Seg {
                    ax: w[0],
                    ay: w[1],
                    bx: w[2],
                    by: w[3],
                })
                .collect(),
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
        self.walls
            .iter()
            .map(|seg| seg.ray_intersect(wx, wy, c, s))
            .fold(max_dist, f32::min)
    }

    /// Index of the next waypoint ahead of `pos`, or None if list is empty.
    pub fn next_waypoint(&self, pos: (f32, f32), last_reached: usize) -> Option<usize> {
        if self.waypoints.is_empty() {
            return None;
        }
        Some((last_reached + 1) % self.waypoints.len())
    }
}
