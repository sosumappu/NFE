#![allow(dead_code)]

use std::{cmp::Reverse, collections::VecDeque};

use nfe_core::Point2;

use crate::raceline::contour::{longest_closed_contour, ClosedContour, ContourSegment};
use crate::raceline::edt::edt_from_sites;
use crate::raceline::grid::BinaryMask;

pub(crate) const TOP_TWO_MARKER_DOMINANCE_FRACTION: f32 = 0.80;
pub(crate) const MIN_MARKER_SHARED_BORDER_CELLS: usize = 4;
const PLATEAU_EPS_M: f32 = 1.0e-5;
const SADDLE_SCORE_TIE_EPS_M: f32 = 1.0e-5;

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct WatershedParams {
    pub min_marker_shared_border_cells: usize,
    pub top_two_marker_dominance_fraction: f32,
}

impl Default for WatershedParams {
    fn default() -> Self {
        Self {
            min_marker_shared_border_cells: MIN_MARKER_SHARED_BORDER_CELLS,
            top_two_marker_dominance_fraction: TOP_TWO_MARKER_DOMINANCE_FRACTION,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub enum WatershedError {
    NoFreeComponent,
    InsufficientMarkers {
        qualifying_components: usize,
    },
    MarkerAmbiguity {
        shared_borders: Vec<usize>,
        top_two_shared_border: usize,
        total_shared_border: usize,
        dominance_fraction: f32,
    },
    AmbiguousCell,
    SaddleTie,
    BranchedPlateau,
    NoClosedContour,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct MarkerSelection {
    pub marker_a_sites: Vec<bool>,
    pub marker_b_sites: Vec<bool>,
    pub shared_borders: Vec<usize>,
    pub dominance_fraction: f32,
}

#[derive(Clone, Debug)]
struct Component {
    cells: Vec<usize>,
    shared_border: usize,
}

pub(crate) fn extract_separatrix(
    mask: &BinaryMask,
    params: WatershedParams,
) -> Result<ClosedContour, WatershedError> {
    let markers = select_two_markers(mask, params)?;
    let field_a = edt_from_sites(mask.spec, &markers.marker_a_sites);
    let field_b = edt_from_sites(mask.spec, &markers.marker_b_sites);
    let phi: Vec<_> = field_a
        .distances_m
        .iter()
        .zip(field_b.distances_m.iter())
        .map(|(a, b)| a - b)
        .collect();
    let clearance: Vec<_> = field_a
        .distances_m
        .iter()
        .zip(field_b.distances_m.iter())
        .map(|(a, b)| a.min(*b))
        .collect();
    let segments = separatrix_segments(mask, &phi, &clearance)?;
    longest_closed_contour(&segments).map_err(|_| WatershedError::NoClosedContour)
}

pub(crate) fn select_two_markers(
    mask: &BinaryMask,
    params: WatershedParams,
) -> Result<MarkerSelection, WatershedError> {
    let main_free = main_free_component(mask).ok_or(WatershedError::NoFreeComponent)?;
    let mut components = blocked_components_adjacent_to(mask, &main_free);
    components.retain(|component| component.shared_border >= params.min_marker_shared_border_cells);
    components.sort_by_key(|component| Reverse(component.shared_border));

    if components.len() < 2 {
        return Err(WatershedError::InsufficientMarkers {
            qualifying_components: components.len(),
        });
    }

    let shared_borders: Vec<_> = components
        .iter()
        .map(|component| component.shared_border)
        .collect();
    let top_two_shared_border = shared_borders[0] + shared_borders[1];
    let total_shared_border: usize = shared_borders.iter().sum();
    let dominance_fraction = top_two_shared_border as f32 / total_shared_border as f32;
    if dominance_fraction < params.top_two_marker_dominance_fraction {
        return Err(WatershedError::MarkerAmbiguity {
            shared_borders,
            top_two_shared_border,
            total_shared_border,
            dominance_fraction,
        });
    }

    let marker_a_sites = sites_for_component(mask.cells().len(), &components[0]);
    let marker_b_sites = sites_for_component(mask.cells().len(), &components[1]);
    Ok(MarkerSelection {
        marker_a_sites,
        marker_b_sites,
        shared_borders,
        dominance_fraction,
    })
}

fn sites_for_component(len: usize, component: &Component) -> Vec<bool> {
    let mut sites = vec![false; len];
    for idx in &component.cells {
        sites[*idx] = true;
    }
    sites
}

fn main_free_component(mask: &BinaryMask) -> Option<Vec<bool>> {
    let mut seen = vec![false; mask.cells().len()];
    let mut best = Vec::new();
    for idx in 0..mask.cells().len() {
        if seen[idx] || !mask.cells()[idx] {
            continue;
        }
        let component = flood_component(mask, idx, true, &mut seen);
        if component.len() > best.len() {
            best = component;
        }
    }
    if best.is_empty() {
        return None;
    }
    let mut out = vec![false; mask.cells().len()];
    for idx in best {
        out[idx] = true;
    }
    Some(out)
}

fn blocked_components_adjacent_to(mask: &BinaryMask, free_component: &[bool]) -> Vec<Component> {
    let mut seen = vec![false; mask.cells().len()];
    let mut out = Vec::new();
    for idx in 0..mask.cells().len() {
        if seen[idx] || mask.cells()[idx] {
            continue;
        }
        let cells = flood_component(mask, idx, false, &mut seen);
        let shared_border = shared_border_with(mask, &cells, free_component);
        if shared_border > 0 {
            out.push(Component {
                cells,
                shared_border,
            });
        }
    }
    out
}

fn flood_component(
    mask: &BinaryMask,
    start: usize,
    target_free: bool,
    seen: &mut [bool],
) -> Vec<usize> {
    let mut out = Vec::new();
    let mut queue = VecDeque::new();
    seen[start] = true;
    queue.push_back(start);
    while let Some(idx) = queue.pop_front() {
        out.push(idx);
        let x = idx % mask.spec.width;
        let y = idx / mask.spec.width;
        for (nx, ny) in four_neighbors(mask, x, y) {
            let nidx = mask.spec.index(nx, ny);
            if !seen[nidx] && mask.cells()[nidx] == target_free {
                seen[nidx] = true;
                queue.push_back(nidx);
            }
        }
    }
    out
}

fn shared_border_with(
    mask: &BinaryMask,
    blocked_cells: &[usize],
    free_component: &[bool],
) -> usize {
    let mut count = 0;
    for idx in blocked_cells {
        let x = idx % mask.spec.width;
        let y = idx / mask.spec.width;
        for (nx, ny) in four_neighbors(mask, x, y) {
            if free_component[mask.spec.index(nx, ny)] {
                count += 1;
            }
        }
    }
    count
}

fn four_neighbors(mask: &BinaryMask, x: usize, y: usize) -> Vec<(usize, usize)> {
    [(1, 0), (-1, 0), (0, 1), (0, -1)]
        .into_iter()
        .filter_map(|(dx, dy)| {
            let nx = x as i32 + dx;
            let ny = y as i32 + dy;
            mask.spec
                .in_bounds_i32(nx, ny)
                .then_some((nx as usize, ny as usize))
        })
        .collect()
}

fn separatrix_segments(
    mask: &BinaryMask,
    phi: &[f32],
    clearance: &[f32],
) -> Result<Vec<ContourSegment>, WatershedError> {
    // Validate plateau topology before marching through zero-valued corners.
    // Simple plateau chains are represented by the zero-corner marching logic;
    // branched plateaus remain ambiguous and are rejected here.
    let _ = plateau_segments(mask, phi)?;
    let mut segments = Vec::new();
    if mask.spec.width < 2 || mask.spec.height < 2 {
        return Ok(segments);
    }
    for y in 0..mask.spec.height - 1 {
        for x in 0..mask.spec.width - 1 {
            segments.extend(cell_segments(mask, phi, clearance, x, y)?);
        }
    }
    Ok(segments)
}

#[derive(Clone, Copy, Debug)]
struct Corner {
    p: Point2,
    local_x: f32,
    local_y: f32,
    phi: f32,
    clearance: f32,
}

#[derive(Clone, Copy, Debug)]
struct Crossing {
    edge: u8,
    p: Point2,
    local_x: f32,
    local_y: f32,
}

fn cell_segments(
    mask: &BinaryMask,
    phi: &[f32],
    clearance: &[f32],
    x: usize,
    y: usize,
) -> Result<Vec<ContourSegment>, WatershedError> {
    if !cell_all_free(mask, x, y) {
        return Ok(Vec::new());
    }
    let corners = cell_corners(mask, phi, clearance, x, y);

    let edges = [(0usize, 1usize), (1, 2), (3, 2), (0, 3)];
    let mut crossings = Vec::new();
    for (edge_idx, (a, b)) in edges.iter().enumerate() {
        if corners[*a].phi.signum() == corners[*b].phi.signum() {
            continue;
        }
        crossings.push(edge_crossing(edge_idx as u8, corners[*a], corners[*b]));
    }

    match crossings.len() {
        0 => Ok(Vec::new()),
        2 => Ok(segment(crossings[0].p, crossings[1].p)
            .into_iter()
            .collect()),
        4 => saddle_segments(&corners, &crossings),
        _ => Err(WatershedError::AmbiguousCell),
    }
}

fn cell_all_free(mask: &BinaryMask, x: usize, y: usize) -> bool {
    mask.is_free(x, y)
        && mask.is_free(x + 1, y)
        && mask.is_free(x + 1, y + 1)
        && mask.is_free(x, y + 1)
}

fn cell_corners(
    mask: &BinaryMask,
    phi: &[f32],
    clearance: &[f32],
    x: usize,
    y: usize,
) -> [Corner; 4] {
    let corner = |cx: usize, cy: usize, local_x: f32, local_y: f32| {
        let idx = mask.spec.index(cx, cy);
        Corner {
            p: cell_center(mask, cx, cy),
            local_x,
            local_y,
            phi: phi[idx],
            clearance: clearance[idx],
        }
    };
    [
        corner(x, y, 0.0, 0.0),
        corner(x + 1, y, 1.0, 0.0),
        corner(x + 1, y + 1, 1.0, 1.0),
        corner(x, y + 1, 0.0, 1.0),
    ]
}

fn cell_center(mask: &BinaryMask, x: usize, y: usize) -> Point2 {
    Point2::new(
        mask.spec.origin.x + (x as f32 + 0.5) * mask.spec.resolution_m,
        mask.spec.origin.y + (y as f32 + 0.5) * mask.spec.resolution_m,
    )
}

fn edge_crossing(edge: u8, a: Corner, b: Corner) -> Crossing {
    let denom = a.phi - b.phi;
    let t = if denom.abs() <= f32::EPSILON {
        0.5
    } else {
        (a.phi / denom).clamp(0.0, 1.0)
    };
    Crossing {
        edge,
        p: Point2::new(a.p.x + (b.p.x - a.p.x) * t, a.p.y + (b.p.y - a.p.y) * t),
        local_x: a.local_x + (b.local_x - a.local_x) * t,
        local_y: a.local_y + (b.local_y - a.local_y) * t,
    }
}

fn saddle_segments(
    corners: &[Corner; 4],
    crossings: &[Crossing],
) -> Result<Vec<ContourSegment>, WatershedError> {
    let crossing = |edge| {
        crossings
            .iter()
            .find(|crossing| crossing.edge == edge)
            .copied()
    };
    let Some(top) = crossing(0) else {
        return Err(WatershedError::AmbiguousCell);
    };
    let Some(right) = crossing(1) else {
        return Err(WatershedError::AmbiguousCell);
    };
    let Some(bottom) = crossing(2) else {
        return Err(WatershedError::AmbiguousCell);
    };
    let Some(left) = crossing(3) else {
        return Err(WatershedError::AmbiguousCell);
    };

    let pairing_a = [(top, right), (bottom, left)];
    let pairing_b = [(top, left), (right, bottom)];
    let score_a = pairing_score(corners, &pairing_a);
    let score_b = pairing_score(corners, &pairing_b);
    if (score_a - score_b).abs() <= SADDLE_SCORE_TIE_EPS_M {
        return Err(WatershedError::SaddleTie);
    }
    let chosen = if score_a > score_b {
        pairing_a
    } else {
        pairing_b
    };
    Ok(chosen
        .into_iter()
        .filter_map(|(a, b)| segment(a.p, b.p))
        .collect())
}

fn pairing_score(corners: &[Corner; 4], pairing: &[(Crossing, Crossing); 2]) -> f32 {
    pairing
        .iter()
        .map(|(a, b)| {
            let x = 0.5 * (a.local_x + b.local_x);
            let y = 0.5 * (a.local_y + b.local_y);
            bilinear_clearance(corners, x, y)
        })
        .fold(f32::INFINITY, f32::min)
}

fn bilinear_clearance(corners: &[Corner; 4], x: f32, y: f32) -> f32 {
    let top = corners[0].clearance * (1.0 - x) + corners[1].clearance * x;
    let bottom = corners[3].clearance * (1.0 - x) + corners[2].clearance * x;
    top * (1.0 - y) + bottom * y
}

fn segment(a: Point2, b: Point2) -> Option<ContourSegment> {
    ContourSegment::new(a, b)
}

fn plateau_segments(mask: &BinaryMask, phi: &[f32]) -> Result<Vec<ContourSegment>, WatershedError> {
    let plateau: Vec<_> = phi
        .iter()
        .zip(mask.cells().iter())
        .map(|(value, free)| *free && value.abs() <= PLATEAU_EPS_M)
        .collect();
    let mut seen = vec![false; plateau.len()];
    let mut segments = Vec::new();
    for idx in 0..plateau.len() {
        if seen[idx] || !plateau[idx] {
            continue;
        }
        let component = plateau_component(mask, &plateau, idx, &mut seen);
        if component.len() < 2 || !plateau_separates_signs(mask, phi, &component) {
            continue;
        }
        segments.extend(collapse_plateau_component(mask, &plateau, &component)?);
    }
    Ok(segments)
}

fn plateau_component(
    mask: &BinaryMask,
    plateau: &[bool],
    start: usize,
    seen: &mut [bool],
) -> Vec<usize> {
    let mut out = Vec::new();
    let mut queue = VecDeque::new();
    seen[start] = true;
    queue.push_back(start);
    while let Some(idx) = queue.pop_front() {
        out.push(idx);
        let x = idx % mask.spec.width;
        let y = idx / mask.spec.width;
        for (nx, ny) in four_neighbors(mask, x, y) {
            let nidx = mask.spec.index(nx, ny);
            if plateau[nidx] && !seen[nidx] {
                seen[nidx] = true;
                queue.push_back(nidx);
            }
        }
    }
    out
}

fn plateau_separates_signs(mask: &BinaryMask, phi: &[f32], component: &[usize]) -> bool {
    let mut has_neg = false;
    let mut has_pos = false;
    for idx in component {
        let x = idx % mask.spec.width;
        let y = idx / mask.spec.width;
        for (nx, ny) in four_neighbors(mask, x, y) {
            let value = phi[mask.spec.index(nx, ny)];
            has_neg |= value < -PLATEAU_EPS_M;
            has_pos |= value > PLATEAU_EPS_M;
        }
    }
    has_neg && has_pos
}

fn collapse_plateau_component(
    mask: &BinaryMask,
    plateau: &[bool],
    component: &[usize],
) -> Result<Vec<ContourSegment>, WatershedError> {
    let mut component_member = vec![false; plateau.len()];
    for idx in component {
        component_member[*idx] = true;
    }
    let mut degrees = Vec::with_capacity(component.len());
    for idx in component {
        let x = idx % mask.spec.width;
        let y = idx / mask.spec.width;
        let degree = four_neighbors(mask, x, y)
            .into_iter()
            .filter(|(nx, ny)| component_member[mask.spec.index(*nx, *ny)])
            .count();
        if degree > 2 {
            return Err(WatershedError::BranchedPlateau);
        }
        degrees.push((*idx, degree));
    }
    let endpoints: Vec<_> = degrees
        .iter()
        .filter_map(|(idx, degree)| (*degree == 1).then_some(*idx))
        .collect();
    if !(endpoints.len() == 2 || endpoints.is_empty()) {
        return Err(WatershedError::BranchedPlateau);
    }

    let start = endpoints.first().copied().unwrap_or(component[0]);
    let ordered = order_plateau(mask, &component_member, start, endpoints.is_empty());
    let mut segments = Vec::new();
    for pair in ordered.windows(2) {
        if let Some(segment) = segment(index_point(mask, pair[0]), index_point(mask, pair[1])) {
            segments.push(segment);
        }
    }
    if endpoints.is_empty() && ordered.len() > 2 {
        if let Some(segment) = segment(
            index_point(mask, *ordered.last().unwrap()),
            index_point(mask, ordered[0]),
        ) {
            segments.push(segment);
        }
    }
    Ok(segments)
}

fn order_plateau(mask: &BinaryMask, member: &[bool], start: usize, cycle: bool) -> Vec<usize> {
    let mut ordered = Vec::new();
    let mut prev = None;
    let mut current = start;
    loop {
        ordered.push(current);
        let x = current % mask.spec.width;
        let y = current / mask.spec.width;
        let next = four_neighbors(mask, x, y)
            .into_iter()
            .map(|(nx, ny)| mask.spec.index(nx, ny))
            .find(|idx| member[*idx] && Some(*idx) != prev && (!ordered.contains(idx) || cycle));
        let Some(next) = next else { break };
        if cycle && next == start {
            break;
        }
        prev = Some(current);
        current = next;
    }
    ordered
}

fn index_point(mask: &BinaryMask, idx: usize) -> Point2 {
    cell_center(mask, idx % mask.spec.width, idx / mask.spec.width)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::raceline::grid::GridSpec;

    fn mask_from_fn(width: usize, height: usize, f: impl Fn(usize, usize) -> bool) -> BinaryMask {
        let spec = GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width,
            height,
        };
        let mut free = vec![false; spec.len()];
        for y in 0..height {
            for x in 0..width {
                free[spec.index(x, y)] = f(x, y);
            }
        }
        BinaryMask::new(spec, free)
    }

    #[test]
    fn clean_annular_track_extracts_closed_separatrix() {
        let center = Point2::new(20.5, 20.5);
        let mask = mask_from_fn(41, 41, |x, y| {
            let p = Point2::new(x as f32 + 0.5, y as f32 + 0.5);
            let r = p.dist(&center);
            (7.0..=15.0).contains(&r)
        });

        let contour = extract_separatrix(&mask, WatershedParams::default()).unwrap();

        assert!(contour.points.len() > 20);
        assert!(contour.length_m > 50.0);
        let mean_radius = contour.points.iter().map(|p| p.dist(&center)).sum::<f32>()
            / contour.points.len() as f32;
        assert!(
            (10.0..=12.5).contains(&mean_radius),
            "mean_radius={mean_radius} len={} points={}",
            contour.length_m,
            contour.points.len()
        );
    }

    #[test]
    fn branch_like_third_marker_fails_dominance_by_construction() {
        let mask = mask_from_fn(20, 14, |x, y| {
            let in_outer_free_rect = (2..18).contains(&x) && (2..12).contains(&y);
            let in_inner_marker = (4..9).contains(&x) && (5..9).contains(&y);
            let in_branch_marker = (11..16).contains(&x) && (5..9).contains(&y);
            in_outer_free_rect && !in_inner_marker && !in_branch_marker
        });

        let err = select_two_markers(&mask, WatershedParams::default()).unwrap_err();

        let WatershedError::MarkerAmbiguity {
            shared_borders,
            top_two_shared_border,
            total_shared_border,
            dominance_fraction,
        } = err
        else {
            panic!("expected marker ambiguity");
        };
        // Counts are fixed by construction:
        // outer rectangle 16x10 -> 2*16 + 2*10 = 52 shared-border edges,
        // each 5x4 internal blocked marker -> 2*5 + 2*4 = 18 edges.
        assert_eq!(shared_borders, vec![52, 18, 18]);
        assert_eq!(top_two_shared_border, 70);
        assert_eq!(total_shared_border, 88);
        assert!(dominance_fraction < TOP_TWO_MARKER_DOMINANCE_FRACTION);
    }

    #[test]
    fn fewer_than_two_markers_returns_ambiguity() {
        let mask = mask_from_fn(8, 8, |x, y| (2..6).contains(&x) && (2..6).contains(&y));

        let err = select_two_markers(&mask, WatershedParams::default()).unwrap_err();

        assert_eq!(
            err,
            WatershedError::InsufficientMarkers {
                qualifying_components: 1
            }
        );
    }

    #[test]
    fn saddle_decider_prefers_higher_clearance_pairing() {
        let corners = [
            Corner {
                p: Point2::new(0.0, 0.0),
                local_x: 0.0,
                local_y: 0.0,
                phi: 3.0,
                clearance: 1.0,
            },
            Corner {
                p: Point2::new(1.0, 0.0),
                local_x: 1.0,
                local_y: 0.0,
                phi: -1.0,
                clearance: 5.0,
            },
            Corner {
                p: Point2::new(1.0, 1.0),
                local_x: 1.0,
                local_y: 1.0,
                phi: 3.0,
                clearance: 1.0,
            },
            Corner {
                p: Point2::new(0.0, 1.0),
                local_x: 0.0,
                local_y: 1.0,
                phi: -1.0,
                clearance: 5.0,
            },
        ];
        let crossings = vec![
            edge_crossing(0, corners[0], corners[1]),
            edge_crossing(1, corners[1], corners[2]),
            edge_crossing(2, corners[3], corners[2]),
            edge_crossing(3, corners[0], corners[3]),
        ];

        let segments = saddle_segments(&corners, &crossings).unwrap();

        assert_eq!(segments.len(), 2);
        assert!(segments.iter().any(|segment| {
            segment.a == crossings[0].p && segment.b == crossings[1].p
                || segment.a == crossings[1].p && segment.b == crossings[0].p
        }));
        assert!(segments.iter().any(|segment| {
            segment.a == crossings[2].p && segment.b == crossings[3].p
                || segment.a == crossings[3].p && segment.b == crossings[2].p
        }));
    }

    #[test]
    fn saddle_decider_tie_returns_ambiguity() {
        let corners = [
            Corner {
                p: Point2::new(0.0, 0.0),
                local_x: 0.0,
                local_y: 0.0,
                phi: 1.0,
                clearance: 1.0,
            },
            Corner {
                p: Point2::new(1.0, 0.0),
                local_x: 1.0,
                local_y: 0.0,
                phi: -1.0,
                clearance: 1.0,
            },
            Corner {
                p: Point2::new(1.0, 1.0),
                local_x: 1.0,
                local_y: 1.0,
                phi: 1.0,
                clearance: 1.0,
            },
            Corner {
                p: Point2::new(0.0, 1.0),
                local_x: 0.0,
                local_y: 1.0,
                phi: -1.0,
                clearance: 1.0,
            },
        ];
        let crossings = vec![
            edge_crossing(0, corners[0], corners[1]),
            edge_crossing(1, corners[1], corners[2]),
            edge_crossing(2, corners[3], corners[2]),
            edge_crossing(3, corners[0], corners[3]),
        ];

        let err = saddle_segments(&corners, &crossings).unwrap_err();

        assert_eq!(err, WatershedError::SaddleTie);
    }

    #[test]
    fn isolated_zero_vertices_do_not_break_closed_contour() {
        let mask = mask_from_fn(5, 5, |_x, _y| true);
        let phi = vec![
            1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, -1.0, 0.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, 1.0,
            0.0, -1.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
        ];
        let clearance = vec![1.0; phi.len()];

        let segments = separatrix_segments(&mask, &phi, &clearance).unwrap();
        let contour = longest_closed_contour(&segments).unwrap();

        assert!(contour.points.len() >= 8, "contour={contour:?}");
        assert!(contour.length_m > 4.0, "contour={contour:?}");
    }

    #[test]
    fn simple_plateau_chain_collapses_to_segments() {
        let mask = mask_from_fn(3, 4, |_x, _y| true);
        let phi = vec![
            -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0, -1.0, 0.0, 1.0,
        ];

        let segments = plateau_segments(&mask, &phi).unwrap();

        assert_eq!(segments.len(), 3);
    }

    #[test]
    fn branched_plateau_returns_ambiguity() {
        let mask = mask_from_fn(3, 3, |_x, _y| true);
        let phi = vec![1.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, -1.0];

        let err = plateau_segments(&mask, &phi).unwrap_err();

        assert_eq!(err, WatershedError::BranchedPlateau);
    }
}
