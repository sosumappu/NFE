#![allow(dead_code)]

use nfe_core::mapping::DistanceField;

use crate::raceline::grid::{BinaryMask, GridSpec};

const INF: f32 = 1.0e20;

pub(crate) fn edt_to_blocked(mask: &BinaryMask) -> DistanceField {
    let sites: Vec<_> = mask.cells().iter().map(|free| !*free).collect();
    edt_from_sites(mask.spec, &sites)
}

pub(crate) fn edt_from_sites(spec: GridSpec, sites: &[bool]) -> DistanceField {
    assert_eq!(sites.len(), spec.len());
    if !sites.iter().any(|site| *site) {
        return DistanceField {
            origin: spec.origin,
            resolution_m: spec.resolution_m,
            width: spec.width as u32,
            height: spec.height as u32,
            distances_m: vec![f32::INFINITY; spec.len()],
        };
    }

    let mut row_pass = vec![INF; spec.len()];
    for y in 0..spec.height {
        let mut f = vec![INF; spec.width];
        for x in 0..spec.width {
            if sites[spec.index(x, y)] {
                f[x] = 0.0;
            }
        }
        let d = edt_1d(&f);
        for x in 0..spec.width {
            row_pass[spec.index(x, y)] = d[x];
        }
    }

    let mut distances_m = vec![0.0; spec.len()];
    for x in 0..spec.width {
        let mut f = vec![INF; spec.height];
        for y in 0..spec.height {
            f[y] = row_pass[spec.index(x, y)];
        }
        let d = edt_1d(&f);
        for y in 0..spec.height {
            distances_m[spec.index(x, y)] = d[y].sqrt() * spec.resolution_m;
        }
    }

    DistanceField {
        origin: spec.origin,
        resolution_m: spec.resolution_m,
        width: spec.width as u32,
        height: spec.height as u32,
        distances_m,
    }
}

fn edt_1d(f: &[f32]) -> Vec<f32> {
    let n = f.len();
    if n == 0 {
        return Vec::new();
    }

    let mut v = vec![0usize; n];
    let mut z = vec![0.0f32; n + 1];
    let mut k = 0usize;
    v[0] = 0;
    z[0] = f32::NEG_INFINITY;
    z[1] = f32::INFINITY;

    for q in 1..n {
        let mut s = intersection(f, q, v[k]);
        while s <= z[k] {
            if k == 0 {
                break;
            }
            k -= 1;
            s = intersection(f, q, v[k]);
        }
        if s <= z[k] {
            v[0] = q;
            z[0] = f32::NEG_INFINITY;
            z[1] = f32::INFINITY;
            k = 0;
        } else {
            k += 1;
            v[k] = q;
            z[k] = s;
            z[k + 1] = f32::INFINITY;
        }
    }

    let mut out = vec![0.0; n];
    k = 0;
    for (q, out_q) in out.iter_mut().enumerate() {
        while z[k + 1] < q as f32 {
            k += 1;
        }
        let dx = q as f32 - v[k] as f32;
        *out_q = dx * dx + f[v[k]];
    }
    out
}

fn intersection(f: &[f32], q: usize, p: usize) -> f32 {
    let qf = q as f32;
    let pf = p as f32;
    ((f[q] + qf * qf) - (f[p] + pf * pf)) / (2.0 * (qf - pf))
}

#[cfg(test)]
mod tests {
    use super::*;
    use nfe_core::Point2;

    fn spec(width: usize, height: usize) -> GridSpec {
        GridSpec {
            origin: Point2::new(0.0, 0.0),
            resolution_m: 1.0,
            width,
            height,
        }
    }

    #[test]
    fn edt_matches_hand_computed_center_site() {
        let spec = spec(3, 3);
        let mut sites = vec![false; spec.len()];
        sites[spec.index(1, 1)] = true;

        let field = edt_from_sites(spec, &sites);

        let expected = [
            2.0f32.sqrt(),
            1.0,
            2.0f32.sqrt(),
            1.0,
            0.0,
            1.0,
            2.0f32.sqrt(),
            1.0,
            2.0f32.sqrt(),
        ];
        for (actual, expected) in field.distances_m.iter().zip(expected) {
            assert!((actual - expected).abs() < 1.0e-5, "{actual} != {expected}");
        }
    }

    #[test]
    fn edt_to_blocked_uses_blocked_cells_as_sites() {
        let spec = spec(3, 1);
        let mask = BinaryMask::new(spec, vec![true, true, false]);

        let field = edt_to_blocked(&mask);

        assert_eq!(field.distances_m, vec![2.0, 1.0, 0.0]);
    }
}
