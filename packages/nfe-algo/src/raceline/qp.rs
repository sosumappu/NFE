#![allow(dead_code)]

use std::collections::BTreeMap;

use clarabel::algebra::CscMatrix;
use clarabel::solver::{DefaultSettings, DefaultSolver, IPSolver, NonnegativeConeT, SolverStatus};

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct QpSettings {
    pub max_solver_iterations: u32,
}

impl Default for QpSettings {
    fn default() -> Self {
        Self {
            max_solver_iterations: 100,
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LinearInequality {
    pub coefficients: Vec<(usize, f64)>,
    pub upper_bound: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct QuadraticProgram {
    pub variables: usize,
    /// Full symmetric Hessian for Clarabel's `0.5 x' P x + q' x` objective.
    pub hessian_entries: Vec<(usize, usize, f64)>,
    pub linear: Vec<f64>,
    pub inequalities: Vec<LinearInequality>,
}

#[derive(Clone, Debug, PartialEq)]
pub enum QpError {
    DimensionMismatch,
    NonFiniteData,
    SolverSetup,
    SolverStatus(SolverStatus),
}

pub(crate) fn solve_qp(
    problem: &QuadraticProgram,
    settings: QpSettings,
) -> Result<Vec<f64>, QpError> {
    validate_problem(problem)?;

    let p = csc_from_entries(
        problem.variables,
        problem.variables,
        &problem.hessian_entries,
    );
    let a = csc_from_inequalities(problem.variables, &problem.inequalities);
    let b: Vec<_> = problem
        .inequalities
        .iter()
        .map(|constraint| constraint.upper_bound)
        .collect();
    let cones = vec![NonnegativeConeT(problem.inequalities.len())];

    let solver_settings = DefaultSettings {
        verbose: false,
        max_iter: settings.max_solver_iterations.max(1),
        ..DefaultSettings::<f64>::default()
    };

    let mut solver = DefaultSolver::new(&p, &problem.linear, &a, &b, &cones, solver_settings)
        .map_err(|_| QpError::SolverSetup)?;
    solver.solve();

    match solver.solution.status {
        SolverStatus::Solved | SolverStatus::AlmostSolved => Ok(solver.solution.x.clone()),
        status => Err(QpError::SolverStatus(status)),
    }
}

fn validate_problem(problem: &QuadraticProgram) -> Result<(), QpError> {
    if problem.linear.len() != problem.variables {
        return Err(QpError::DimensionMismatch);
    }
    for &(row, col, value) in &problem.hessian_entries {
        if row >= problem.variables || col >= problem.variables || !value.is_finite() {
            return Err(QpError::NonFiniteData);
        }
    }
    for &value in &problem.linear {
        if !value.is_finite() {
            return Err(QpError::NonFiniteData);
        }
    }
    for inequality in &problem.inequalities {
        if !inequality.upper_bound.is_finite() {
            return Err(QpError::NonFiniteData);
        }
        for &(index, coefficient) in &inequality.coefficients {
            if index >= problem.variables || !coefficient.is_finite() {
                return Err(QpError::NonFiniteData);
            }
        }
    }
    Ok(())
}

fn csc_from_entries(rows: usize, cols: usize, entries: &[(usize, usize, f64)]) -> CscMatrix<f64> {
    let mut columns = vec![BTreeMap::<usize, f64>::new(); cols];
    for &(row, col, value) in entries {
        if value == 0.0 {
            continue;
        }
        *columns[col].entry(row).or_insert(0.0) += value;
    }
    csc_from_columns(rows, cols, columns)
}

fn csc_from_inequalities(variables: usize, inequalities: &[LinearInequality]) -> CscMatrix<f64> {
    let mut columns = vec![BTreeMap::<usize, f64>::new(); variables];
    for (row, inequality) in inequalities.iter().enumerate() {
        for &(col, value) in &inequality.coefficients {
            if value == 0.0 {
                continue;
            }
            *columns[col].entry(row).or_insert(0.0) += value;
        }
    }
    csc_from_columns(inequalities.len(), variables, columns)
}

fn csc_from_columns(
    rows: usize,
    cols: usize,
    columns: Vec<BTreeMap<usize, f64>>,
) -> CscMatrix<f64> {
    let mut colptr = Vec::with_capacity(cols + 1);
    let mut rowval = Vec::new();
    let mut nzval = Vec::new();
    colptr.push(0);
    for column in columns {
        for (row, value) in column {
            if value != 0.0 {
                rowval.push(row);
                nzval.push(value);
            }
        }
        colptr.push(rowval.len());
    }
    CscMatrix::new(rows, cols, colptr, rowval, nzval)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn solves_bounded_scalar_qp() {
        let problem = QuadraticProgram {
            variables: 1,
            hessian_entries: vec![(0, 0, 2.0)],
            linear: vec![-2.0],
            inequalities: vec![
                LinearInequality {
                    coefficients: vec![(0, 1.0)],
                    upper_bound: 0.5,
                },
                LinearInequality {
                    coefficients: vec![(0, -1.0)],
                    upper_bound: 1.0,
                },
            ],
        };

        let x = solve_qp(&problem, QpSettings::default()).unwrap();
        assert!((x[0] - 0.5).abs() < 1.0e-5, "{x:?}");
    }
}
