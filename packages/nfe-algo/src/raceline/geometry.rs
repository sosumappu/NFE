#![allow(dead_code)]

use nfe_core::Point2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct UnitVector2 {
    pub x: f32,
    pub y: f32,
}

impl UnitVector2 {
    pub(crate) fn new(x: f32, y: f32) -> Option<Self> {
        let norm = x.hypot(y);
        if norm <= f32::EPSILON || !norm.is_finite() {
            return None;
        }
        Some(Self {
            x: x / norm,
            y: y / norm,
        })
    }

    pub(crate) fn left_normal(self) -> Self {
        Self {
            x: -self.y,
            y: self.x,
        }
    }

    pub(crate) fn right_normal(self) -> Self {
        Self {
            x: self.y,
            y: -self.x,
        }
    }

    pub(crate) fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y
    }
}

pub(crate) fn distance(a: Point2, b: Point2) -> f32 {
    a.dist(&b)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn left_normal_is_perpendicular_and_unit_length() {
        let tangent = UnitVector2::new(3.0, 4.0).unwrap();
        let normal = tangent.left_normal();

        assert!(tangent.dot(normal).abs() < 1.0e-6);
        assert!((normal.x.hypot(normal.y) - 1.0).abs() < 1.0e-6);
    }
}
