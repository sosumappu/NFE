#![allow(dead_code)]

use std::collections::{HashMap, VecDeque};

use nfe_core::Point2;

const STITCH_EPS_M: f32 = 1.0e-4;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ContourSegment {
    pub a: Point2,
    pub b: Point2,
}

impl ContourSegment {
    pub(crate) fn new(a: Point2, b: Point2) -> Option<Self> {
        (a.dist(&b) > STITCH_EPS_M).then_some(Self { a, b })
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct ClosedContour {
    pub points: Vec<Point2>,
    pub length_m: f32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) enum ContourError {
    NoClosedLoop,
}

pub(crate) fn longest_closed_contour(
    segments: &[ContourSegment],
) -> Result<ClosedContour, ContourError> {
    stitch_closed_contours(segments)
        .into_iter()
        .max_by(|a, b| {
            a.length_m
                .partial_cmp(&b.length_m)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .ok_or(ContourError::NoClosedLoop)
}

pub(crate) fn stitch_closed_contours(segments: &[ContourSegment]) -> Vec<ClosedContour> {
    let graph = Graph::from_segments(segments);
    if graph.edges.is_empty() {
        return Vec::new();
    }

    let alive = graph.prune_open_chains();
    graph.extract_cycles(&alive)
}

struct Graph {
    points: Vec<Point2>,
    edges: Vec<(usize, usize)>,
    adjacency: Vec<Vec<usize>>,
}

impl Graph {
    fn from_segments(segments: &[ContourSegment]) -> Self {
        let mut key_to_vertex = HashMap::new();
        let mut points = Vec::new();
        let mut edges = Vec::new();

        for segment in segments {
            let a = vertex_for(segment.a, &mut key_to_vertex, &mut points);
            let b = vertex_for(segment.b, &mut key_to_vertex, &mut points);
            if a != b {
                edges.push((a, b));
            }
        }

        let mut adjacency = vec![Vec::new(); points.len()];
        for (idx, (a, b)) in edges.iter().copied().enumerate() {
            adjacency[a].push(idx);
            adjacency[b].push(idx);
        }

        Self {
            points,
            edges,
            adjacency,
        }
    }

    fn prune_open_chains(&self) -> Vec<bool> {
        let mut alive = vec![true; self.edges.len()];
        let mut degree: Vec<_> = self.adjacency.iter().map(|edges| edges.len()).collect();
        let mut queue: VecDeque<_> = degree
            .iter()
            .enumerate()
            .filter_map(|(idx, degree)| (*degree <= 1).then_some(idx))
            .collect();

        while let Some(vertex) = queue.pop_front() {
            if degree[vertex] > 1 {
                continue;
            }
            let incident: Vec<_> = self.adjacency[vertex]
                .iter()
                .copied()
                .filter(|edge| alive[*edge])
                .collect();
            for edge in incident {
                alive[edge] = false;
                let (a, b) = self.edges[edge];
                for v in [a, b] {
                    degree[v] = degree[v].saturating_sub(1);
                    if degree[v] == 1 {
                        queue.push_back(v);
                    }
                }
            }
        }

        alive
    }

    fn extract_cycles(&self, alive: &[bool]) -> Vec<ClosedContour> {
        let mut used = vec![false; self.edges.len()];
        let mut out = Vec::new();
        for start_edge in 0..self.edges.len() {
            if !alive[start_edge] || used[start_edge] {
                continue;
            }
            let Some(cycle) = self.trace_cycle(start_edge, alive, &mut used) else {
                continue;
            };
            if cycle.len() < 3 {
                continue;
            }
            let points: Vec<_> = cycle
                .into_iter()
                .map(|vertex| self.points[vertex])
                .collect();
            let length_m = closed_length(&points);
            if length_m > STITCH_EPS_M {
                out.push(ClosedContour { points, length_m });
            }
        }
        out
    }

    fn trace_cycle(
        &self,
        start_edge: usize,
        alive: &[bool],
        used: &mut [bool],
    ) -> Option<Vec<usize>> {
        let (start, mut current) = self.edges[start_edge];
        let mut prev = start;
        let mut edge = start_edge;
        let mut cycle = vec![start];

        loop {
            used[edge] = true;
            cycle.push(current);
            if current == start {
                cycle.pop();
                return Some(cycle);
            }

            let next_edge = self.adjacency[current]
                .iter()
                .copied()
                .find(|candidate| alive[*candidate] && !used[*candidate] && *candidate != edge)
                .or_else(|| {
                    self.adjacency[current]
                        .iter()
                        .copied()
                        .find(|candidate| alive[*candidate] && *candidate != edge)
                })?;
            let (a, b) = self.edges[next_edge];
            let next = if a == current { b } else { a };
            if next == prev && self.alive_degree(current, alive) > 1 {
                return None;
            }
            prev = current;
            current = next;
            edge = next_edge;
        }
    }

    fn alive_degree(&self, vertex: usize, alive: &[bool]) -> usize {
        self.adjacency[vertex]
            .iter()
            .filter(|edge| alive[**edge])
            .count()
    }
}

fn vertex_for(
    point: Point2,
    key_to_vertex: &mut HashMap<PointKey, usize>,
    points: &mut Vec<Point2>,
) -> usize {
    let key = PointKey::new(point);
    if let Some(idx) = key_to_vertex.get(&key) {
        return *idx;
    }
    let idx = points.len();
    key_to_vertex.insert(key, idx);
    points.push(point);
    idx
}

fn closed_length(points: &[Point2]) -> f32 {
    if points.len() < 2 {
        return 0.0;
    }
    let open_len: f32 = points.windows(2).map(|w| w[0].dist(&w[1])).sum();
    open_len + points[points.len() - 1].dist(&points[0])
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct PointKey(i64, i64);

impl PointKey {
    fn new(point: Point2) -> Self {
        Self(
            (point.x / STITCH_EPS_M).round() as i64,
            (point.y / STITCH_EPS_M).round() as i64,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stitches_square_segments_into_closed_contour() {
        let segments = vec![
            ContourSegment::new(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)).unwrap(),
            ContourSegment::new(Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)).unwrap(),
            ContourSegment::new(Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)).unwrap(),
            ContourSegment::new(Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)).unwrap(),
        ];

        let contour = longest_closed_contour(&segments).unwrap();

        assert_eq!(contour.points.len(), 4);
        assert!((contour.length_m - 4.0).abs() < 1.0e-6);
    }

    #[test]
    fn prunes_spur_before_extracting_cycle() {
        let segments = vec![
            ContourSegment::new(Point2::new(0.0, 0.0), Point2::new(1.0, 0.0)).unwrap(),
            ContourSegment::new(Point2::new(1.0, 0.0), Point2::new(1.0, 1.0)).unwrap(),
            ContourSegment::new(Point2::new(1.0, 1.0), Point2::new(0.0, 1.0)).unwrap(),
            ContourSegment::new(Point2::new(0.0, 1.0), Point2::new(0.0, 0.0)).unwrap(),
            ContourSegment::new(Point2::new(1.0, 1.0), Point2::new(2.0, 1.0)).unwrap(),
        ];

        let contour = longest_closed_contour(&segments).unwrap();

        assert_eq!(contour.points.len(), 4);
        assert!((contour.length_m - 4.0).abs() < 1.0e-6);
    }
}
