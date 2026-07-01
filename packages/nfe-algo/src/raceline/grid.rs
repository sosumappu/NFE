#![allow(dead_code)]

use std::collections::VecDeque;

use nfe_core::mapping::OccupancyGrid;
use nfe_core::Point2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct GridSpec {
    pub origin: Point2,
    pub resolution_m: f32,
    pub width: usize,
    pub height: usize,
}

impl GridSpec {
    pub(crate) fn from_occupancy(grid: &OccupancyGrid) -> Self {
        Self {
            origin: grid.origin,
            resolution_m: grid.resolution_m,
            width: grid.width as usize,
            height: grid.height as usize,
        }
    }

    pub(crate) fn len(self) -> usize {
        self.width.saturating_mul(self.height)
    }

    pub(crate) fn index(self, x: usize, y: usize) -> usize {
        y * self.width + x
    }

    pub(crate) fn in_bounds_i32(self, x: i32, y: i32) -> bool {
        x >= 0 && y >= 0 && x < self.width as i32 && y < self.height as i32
    }

    pub(crate) fn world_to_cell(self, point: Point2) -> Option<(usize, usize)> {
        let x = ((point.x - self.origin.x) / self.resolution_m).floor() as i32;
        let y = ((point.y - self.origin.y) / self.resolution_m).floor() as i32;
        self.in_bounds_i32(x, y).then_some((x as usize, y as usize))
    }

    pub(crate) fn cell_center(self, x: usize, y: usize) -> Point2 {
        Point2::new(
            self.origin.x + (x as f32 + 0.5) * self.resolution_m,
            self.origin.y + (y as f32 + 0.5) * self.resolution_m,
        )
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct BinarizeParams {
    pub free_below_log_odds: f32,
}

impl Default for BinarizeParams {
    fn default() -> Self {
        Self {
            free_below_log_odds: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Connectivity {
    Four,
    Eight,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct BinaryMask {
    pub spec: GridSpec,
    free: Vec<bool>,
}

impl BinaryMask {
    pub(crate) fn new(spec: GridSpec, free: Vec<bool>) -> Self {
        assert_eq!(free.len(), spec.len());
        Self { spec, free }
    }

    pub(crate) fn from_occupancy(grid: &OccupancyGrid, params: BinarizeParams) -> Self {
        let spec = GridSpec::from_occupancy(grid);
        let free = grid
            .cells
            .iter()
            .map(|value| *value < 0.0 && *value < params.free_below_log_odds)
            .collect();
        Self::new(spec, free)
    }

    pub(crate) fn cells(&self) -> &[bool] {
        &self.free
    }

    pub(crate) fn is_free(&self, x: usize, y: usize) -> bool {
        self.free[self.spec.index(x, y)]
    }

    pub(crate) fn is_free_i32(&self, x: i32, y: i32) -> bool {
        if !self.spec.in_bounds_i32(x, y) {
            return false;
        }
        self.is_free(x as usize, y as usize)
    }

    pub(crate) fn set_free(&mut self, x: usize, y: usize, value: bool) {
        let idx = self.spec.index(x, y);
        self.free[idx] = value;
    }

    pub(crate) fn is_free_world(&self, point: Point2) -> bool {
        self.spec
            .world_to_cell(point)
            .is_some_and(|(x, y)| self.is_free(x, y))
    }

    pub(crate) fn count_free(&self) -> usize {
        self.free.iter().filter(|cell| **cell).count()
    }

    pub(crate) fn open(&self, radius_cells: usize) -> Self {
        if radius_cells == 0 {
            return self.clone();
        }
        self.erode(radius_cells).dilate(radius_cells)
    }

    pub(crate) fn close(&self, radius_cells: usize) -> Self {
        if radius_cells == 0 {
            return self.clone();
        }
        self.dilate(radius_cells).erode(radius_cells)
    }

    pub(crate) fn retain_largest_free_component(&self, connectivity: Connectivity) -> Self {
        self.filter_free_components(usize::MAX, connectivity)
    }

    pub(crate) fn remove_free_components_smaller_than(
        &self,
        min_cells: usize,
        connectivity: Connectivity,
    ) -> Self {
        self.filter_free_components(min_cells, connectivity)
    }

    fn erode(&self, radius_cells: usize) -> Self {
        let r = radius_cells as i32;
        let mut out = vec![false; self.free.len()];
        for y in 0..self.spec.height {
            for x in 0..self.spec.width {
                let mut keep = true;
                'window: for dy in -r..=r {
                    for dx in -r..=r {
                        if !self.is_free_i32(x as i32 + dx, y as i32 + dy) {
                            keep = false;
                            break 'window;
                        }
                    }
                }
                out[self.spec.index(x, y)] = keep;
            }
        }
        Self::new(self.spec, out)
    }

    fn dilate(&self, radius_cells: usize) -> Self {
        let r = radius_cells as i32;
        let mut out = vec![false; self.free.len()];
        for y in 0..self.spec.height {
            for x in 0..self.spec.width {
                let mut any = false;
                'window: for dy in -r..=r {
                    for dx in -r..=r {
                        if self.is_free_i32(x as i32 + dx, y as i32 + dy) {
                            any = true;
                            break 'window;
                        }
                    }
                }
                out[self.spec.index(x, y)] = any;
            }
        }
        Self::new(self.spec, out)
    }

    fn filter_free_components(&self, min_cells: usize, connectivity: Connectivity) -> Self {
        let components = self.free_components(connectivity);
        if components.is_empty() {
            return self.clone();
        }

        let keep = if min_cells == usize::MAX {
            let largest = components
                .iter()
                .enumerate()
                .max_by_key(|(_, component)| component.len())
                .map(|(idx, _)| idx)
                .unwrap_or(0);
            components
                .into_iter()
                .enumerate()
                .filter_map(|(idx, component)| (idx == largest).then_some(component))
                .flatten()
                .collect::<Vec<_>>()
        } else {
            components
                .into_iter()
                .filter(|component| component.len() >= min_cells)
                .flatten()
                .collect::<Vec<_>>()
        };

        let mut out = vec![false; self.free.len()];
        for idx in keep {
            out[idx] = true;
        }
        Self::new(self.spec, out)
    }

    fn free_components(&self, connectivity: Connectivity) -> Vec<Vec<usize>> {
        let mut seen = vec![false; self.free.len()];
        let mut out = Vec::new();
        for idx in 0..self.free.len() {
            if seen[idx] || !self.free[idx] {
                continue;
            }
            let mut component = Vec::new();
            let mut queue = VecDeque::new();
            seen[idx] = true;
            queue.push_back(idx);
            while let Some(cell) = queue.pop_front() {
                component.push(cell);
                let x = cell % self.spec.width;
                let y = cell / self.spec.width;
                for (nx, ny) in self.neighbors(x, y, connectivity) {
                    let nidx = self.spec.index(nx, ny);
                    if !seen[nidx] && self.free[nidx] {
                        seen[nidx] = true;
                        queue.push_back(nidx);
                    }
                }
            }
            out.push(component);
        }
        out
    }

    fn neighbors(&self, x: usize, y: usize, connectivity: Connectivity) -> Vec<(usize, usize)> {
        let offsets: &[(i32, i32)] = match connectivity {
            Connectivity::Four => &[(1, 0), (-1, 0), (0, 1), (0, -1)],
            Connectivity::Eight => &[
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (-1, 1),
                (1, -1),
                (-1, -1),
            ],
        };
        offsets
            .iter()
            .filter_map(|(dx, dy)| {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                self.spec
                    .in_bounds_i32(nx, ny)
                    .then_some((nx as usize, ny as usize))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn grid(width: u32, height: u32, cells: Vec<f32>) -> OccupancyGrid {
        OccupancyGrid {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width,
            height,
            cells,
        }
    }

    #[test]
    fn binarization_treats_unknown_as_blocked() {
        let mask = BinaryMask::from_occupancy(
            &grid(3, 1, vec![-0.5, 0.0, 0.5]),
            BinarizeParams::default(),
        );

        assert!(mask.is_free(0, 0));
        assert!(!mask.is_free(1, 0));
        assert!(!mask.is_free(2, 0));
    }

    #[test]
    fn opening_removes_isolated_free_artifact() {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width: 5,
            height: 5,
        };
        let mut mask = BinaryMask::new(spec, vec![false; spec.len()]);
        mask.set_free(2, 2, true);

        let opened = mask.open(1);

        assert!(!opened.is_free(2, 2));
        assert_eq!(opened.count_free(), 0);
    }

    #[test]
    fn closing_fills_isolated_blocked_hole() {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width: 5,
            height: 5,
        };
        let mut mask = BinaryMask::new(spec, vec![false; spec.len()]);
        for y in 1..4 {
            for x in 1..4 {
                mask.set_free(x, y, true);
            }
        }
        mask.set_free(2, 2, false);

        let closed = mask.close(1);

        assert!(closed.is_free(2, 2));
    }

    #[test]
    fn component_filter_removes_isolated_blob() {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width: 5,
            height: 3,
        };
        let mut mask = BinaryMask::new(spec, vec![false; spec.len()]);
        mask.set_free(0, 0, true);
        mask.set_free(1, 0, true);
        mask.set_free(0, 1, true);
        mask.set_free(4, 2, true);

        let filtered = mask.remove_free_components_smaller_than(2, Connectivity::Four);

        assert!(filtered.is_free(0, 0));
        assert!(filtered.is_free(1, 0));
        assert!(filtered.is_free(0, 1));
        assert!(!filtered.is_free(4, 2));
    }
}
