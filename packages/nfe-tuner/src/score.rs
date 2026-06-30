use crate::evaluate::SimEpisodeScore;

#[derive(Clone, Copy, Debug, serde::Deserialize, serde::Serialize, PartialEq)]
pub struct CandidateScore {
    pub lap_time_s: f64,
    pub off_track_count: u32,
    pub score: f64,
}

impl CandidateScore {
    pub fn from_sim_episode(score: SimEpisodeScore) -> Self {
        Self {
            lap_time_s: score.finish_time_s.map(f64::from).unwrap_or(0.0),
            off_track_count: u32::from(score.crashed),
            score: score.cost,
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
            lap_time_s: 12.5,
            off_track_count: 1,
            score: 0.7,
        };

        let json = serde_json::to_string(&score).unwrap();
        let decoded: CandidateScore = serde_json::from_str(&json).unwrap();

        assert_eq!(decoded, score);
    }
}
