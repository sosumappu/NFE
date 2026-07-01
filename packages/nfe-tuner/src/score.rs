use crate::evaluate::SimEpisodeScore;

pub const INVALID_CANDIDATE_SCORE: f64 = 1.0e9;

#[derive(Clone, Debug, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct CandidateScore {
    pub status: String,
    pub lap_time_s: f64,
    pub off_track_count: u32,
    pub score: f64,
    pub completed_laps: u32,
    pub progress_m: f64,
    pub progress_ratio: f64,
    pub avg_speed_ms: f64,
    pub max_speed_ms: f64,
    pub lateral_rms_m: f64,
    pub heading_rms_rad: f64,
    pub steering_rate_rms: f64,
    pub throttle_rate_rms: f64,
    pub unavailable_fraction: f64,
    pub ticks: u64,
    pub crashed: bool,
    pub error: Option<String>,
}

impl CandidateScore {
    pub fn from_sim_episode(score: SimEpisodeScore) -> Self {
        Self {
            status: if score.crashed {
                "crashed".to_string()
            } else if score.finish_time_s.is_some() {
                "finished".to_string()
            } else {
                "incomplete".to_string()
            },
            lap_time_s: score.finish_time_s.map(f64::from).unwrap_or(0.0),
            off_track_count: u32::from(score.crashed),
            score: score.cost,
            completed_laps: score.completed_laps,
            progress_m: f64::from(score.progress_m),
            progress_ratio: f64::from(score.progress_ratio),
            avg_speed_ms: f64::from(score.avg_speed_ms),
            max_speed_ms: f64::from(score.max_speed_ms),
            lateral_rms_m: f64::from(score.lateral_rms_m),
            heading_rms_rad: f64::from(score.heading_rms_rad),
            steering_rate_rms: f64::from(score.steering_rate_rms),
            throttle_rate_rms: f64::from(score.throttle_rate_rms),
            unavailable_fraction: f64::from(score.unavailable_fraction),
            ticks: score.ticks,
            crashed: score.crashed,
            error: None,
        }
    }

    pub fn invalid(error: impl Into<String>) -> Self {
        Self {
            status: "invalid".to_string(),
            lap_time_s: 0.0,
            off_track_count: 0,
            score: INVALID_CANDIDATE_SCORE,
            completed_laps: 0,
            progress_m: 0.0,
            progress_ratio: 0.0,
            avg_speed_ms: 0.0,
            max_speed_ms: 0.0,
            lateral_rms_m: 0.0,
            heading_rms_rad: 0.0,
            steering_rate_rms: 0.0,
            throttle_rate_rms: 0.0,
            unavailable_fraction: 0.0,
            ticks: 0,
            crashed: false,
            error: Some(error.into()),
        }
    }

    pub fn replay(score: f64, ticks: u64, error: Option<String>) -> Self {
        Self {
            status: if error.is_some() {
                "invalid".to_string()
            } else if ticks > 0 {
                "evaluated".to_string()
            } else {
                "empty".to_string()
            },
            lap_time_s: 0.0,
            off_track_count: 0,
            score,
            completed_laps: 0,
            progress_m: 0.0,
            progress_ratio: 0.0,
            avg_speed_ms: 0.0,
            max_speed_ms: 0.0,
            lateral_rms_m: 0.0,
            heading_rms_rad: 0.0,
            steering_rate_rms: 0.0,
            throttle_rate_rms: 0.0,
            unavailable_fraction: 0.0,
            ticks,
            crashed: false,
            error,
        }
    }
}

impl From<SimEpisodeScore> for CandidateScore {
    fn from(score: SimEpisodeScore) -> Self {
        Self::from_sim_episode(score)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn serializes_round_trip() {
        let score = CandidateScore {
            status: "finished".to_string(),
            lap_time_s: 12.5,
            off_track_count: 1,
            score: 0.7,
            completed_laps: 3,
            progress_m: 42.0,
            progress_ratio: 1.0,
            avg_speed_ms: 2.1,
            max_speed_ms: 3.0,
            lateral_rms_m: 0.1,
            heading_rms_rad: 0.2,
            steering_rate_rms: 0.3,
            throttle_rate_rms: 0.4,
            unavailable_fraction: 0.0,
            ticks: 1200,
            crashed: false,
            error: None,
        };

        let json = serde_json::to_string(&score).unwrap();
        let decoded: CandidateScore = serde_json::from_str(&json).unwrap();

        assert_eq!(decoded, score);
    }

    #[test]
    fn invalid_scores_are_penalized() {
        let score = CandidateScore::invalid("bad bounds");

        assert_eq!(score.status, "invalid");
        assert_eq!(score.score, INVALID_CANDIDATE_SCORE);
        assert!(score.error.unwrap().contains("bad bounds"));
    }
}
