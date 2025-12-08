use crate::graph::EdgeIndex;
use geo::Point;

#[derive(Debug, Clone)]
pub struct EdgeCand {
    pub edge: Option<EdgeIndex>,
    pub point: Option<Point<f64>>, // Optional as sometimes point is not set?
    pub pen: f64,
    pub time: f64,
    pub progr: f64,
    pub dep_prede: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct EdgeHop {
    pub edges: Vec<EdgeIndex>, // Path of edges
    pub start_edge: Option<EdgeIndex>,
    pub end_edge: Option<EdgeIndex>,
    pub start_progr: f64,
    pub end_progr: f64,
    pub start_point: Option<Point<f64>>,
    pub end_point: Option<Point<f64>>,
}

pub type EdgeCandGroup = Vec<EdgeCand>;
pub type EdgeCandMap = Vec<EdgeCandGroup>;
pub type EdgeListHops = Vec<EdgeHop>;
