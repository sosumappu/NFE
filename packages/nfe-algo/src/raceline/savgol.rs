#![allow(dead_code)]

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct SavitzkyGolayParams {
    pub window_points: usize,
    pub polynomial_order: usize,
}

impl Default for SavitzkyGolayParams {
    fn default() -> Self {
        Self {
            window_points: 7,
            polynomial_order: 3,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SavitzkyGolayError {
    EmptyInput,
    EvenWindow,
    WindowTooSmall,
    SingularFit,
}

pub(crate) fn smooth_circular(
    values: &[f32],
    params: SavitzkyGolayParams,
) -> Result<Vec<f32>, SavitzkyGolayError> {
    if values.is_empty() {
        return Err(SavitzkyGolayError::EmptyInput);
    }
    if params.window_points.is_multiple_of(2) {
        return Err(SavitzkyGolayError::EvenWindow);
    }
    if params.window_points <= params.polynomial_order {
        return Err(SavitzkyGolayError::WindowTooSmall);
    }

    let window = params.window_points.min(if values.len() % 2 == 1 {
        values.len()
    } else {
        values.len().saturating_sub(1)
    });
    if window == 0 || window <= params.polynomial_order || window.is_multiple_of(2) {
        return Err(SavitzkyGolayError::WindowTooSmall);
    }
    let weights = center_weights(window, params.polynomial_order)?;
    let half = window / 2;
    let n = values.len();
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let mut acc = 0.0;
        for (j, weight) in weights.iter().enumerate() {
            let offset = j as isize - half as isize;
            let idx = (i as isize + offset).rem_euclid(n as isize) as usize;
            acc += weight * values[idx];
        }
        out.push(acc);
    }
    Ok(out)
}

fn center_weights(
    window_points: usize,
    polynomial_order: usize,
) -> Result<Vec<f32>, SavitzkyGolayError> {
    let cols = polynomial_order + 1;
    let half = window_points / 2;
    let mut ata = vec![vec![0.0f32; cols]; cols];
    for row in 0..window_points {
        let x = row as i32 - half as i32;
        let powers = powers(x as f32, polynomial_order);
        for i in 0..cols {
            for j in 0..cols {
                ata[i][j] += powers[i] * powers[j];
            }
        }
    }
    let inv = invert(ata).ok_or(SavitzkyGolayError::SingularFit)?;
    let mut weights = Vec::with_capacity(window_points);
    for row in 0..window_points {
        let x = row as i32 - half as i32;
        let powers = powers(x as f32, polynomial_order);
        let weight = (0..cols).map(|k| inv[0][k] * powers[k]).sum();
        weights.push(weight);
    }
    Ok(weights)
}

fn powers(x: f32, order: usize) -> Vec<f32> {
    let mut out = vec![1.0; order + 1];
    for i in 1..=order {
        out[i] = out[i - 1] * x;
    }
    out
}

fn invert(mut a: Vec<Vec<f32>>) -> Option<Vec<Vec<f32>>> {
    let n = a.len();
    let mut inv = vec![vec![0.0; n]; n];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }

    for col in 0..n {
        let pivot = (col..n).max_by(|a_idx, b_idx| {
            a[*a_idx][col]
                .abs()
                .partial_cmp(&a[*b_idx][col].abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })?;
        if a[pivot][col].abs() <= 1.0e-8 {
            return None;
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);

        let denom = a[col][col];
        for j in 0..n {
            a[col][j] /= denom;
            inv[col][j] /= denom;
        }
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = a[row][col];
            for j in 0..n {
                a[row][j] -= factor * a[col][j];
                inv[row][j] -= factor * inv[col][j];
            }
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn smoothing_preserves_constant_signal() {
        let values = vec![2.5; 11];

        let smoothed = smooth_circular(&values, SavitzkyGolayParams::default()).unwrap();

        for value in smoothed {
            assert!((value - 2.5).abs() < 1.0e-5);
        }
    }

    #[test]
    fn smoothing_damps_single_spike() {
        let mut values = vec![0.0; 11];
        values[5] = 10.0;

        let smoothed = smooth_circular(
            &values,
            SavitzkyGolayParams {
                window_points: 5,
                polynomial_order: 2,
            },
        )
        .unwrap();

        assert!(smoothed[5] > 0.0);
        assert!(smoothed[5] < 10.0);
    }
}
