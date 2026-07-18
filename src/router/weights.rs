use crate::graph::TransitInfo;

#[derive(Debug, Clone)]
pub struct RoutingOpts {
    pub transition_pen: f64,
    pub line_unmatched_punish_fact: f64,
    pub line_name_to_unmatched_punish_fact: f64,
    pub line_name_from_unmatched_punish_fact: f64,
    pub full_turn_punish_fac: u32,
    pub full_turn_angle: f64,
    pub turn_restr_cost: u32,
    pub loop_ttl: u32,
    pub max_group_diff_seconds: u32,
}

impl Default for RoutingOpts {
    fn default() -> Self {
        Self {
            transition_pen: 0.0,
            line_unmatched_punish_fact: 1.0,
            line_name_to_unmatched_punish_fact: 1.0,
            line_name_from_unmatched_punish_fact: 1.0,
            full_turn_punish_fac: 0,
            full_turn_angle: 0.0,
            turn_restr_cost: 0,
            loop_ttl: 0,
            max_group_diff_seconds: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LineSimilarity {
    pub name_similar: bool,
    pub from_similar: bool,
    pub to_similar: bool,
}

impl Ord for LineSimilarity {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        let self_sum = self.name_similar as u8 + self.from_similar as u8 + self.to_similar as u8;
        let other_sum =
            other.name_similar as u8 + other.from_similar as u8 + other.to_similar as u8;
        self_sum.cmp(&other_sum)
    }
}

impl PartialOrd for LineSimilarity {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

#[derive(Debug, Clone, Default)]
pub struct RoutingAttrs {
    pub short_name: String,
    pub line_from: String,
    pub line_to: String,
}

impl RoutingAttrs {
    pub fn merge(&mut self, other: &RoutingAttrs) {
        // C++ logic: if empty, take other. If distinct, maybe clear?
        // Original logic not fully visible, assuming accumulation or "best"
        if self.short_name.is_empty() {
            self.short_name = other.short_name.clone();
        }
        if self.line_from.is_empty() {
            self.line_from = other.line_from.clone();
        }
        if self.line_to.is_empty() {
            self.line_to = other.line_to.clone();
        }
    }

    pub fn simi(&self, info: &TransitInfo) -> LineSimilarity {
        LineSimilarity {
            name_similar: self.short_name == info.short_name,
            from_similar: self.line_from == info.from_str,
            to_similar: self.line_to == info.to_str,
        }
    }
}

pub trait TransWeight {
    fn weight(c: u32, d: f64, arr_t: f64, hop_dist: f64, opts: &RoutingOpts) -> f64;
    fn inv_weight(c: f64, opts: &RoutingOpts) -> u32;
    fn max_cost(hop_time: f64, opts: &RoutingOpts) -> u32;

    // Equivalent to static booleans in C++ traits
    fn allows_fast_route() -> bool;
    fn need_dist() -> bool;
}

pub struct ExpoTransWeight;

impl TransWeight for ExpoTransWeight {
    fn weight(c: u32, _d: f64, _arr_t: f64, _hop_dist: f64, opts: &RoutingOpts) -> f64 {
        // C++: rOpts.transitionPen * static_cast<double>(c) / 10.0;
        opts.transition_pen * (c as f64) / 10.0
    }

    fn inv_weight(c: f64, opts: &RoutingOpts) -> u32 {
        // C++: std::round((c / rOpts.transitionPen) * 10.0);
        ((c / opts.transition_pen) * 10.0).round() as u32
    }

    fn max_cost(t_time: f64, opts: &RoutingOpts) -> u32 {
        // C++: ceil(fmax(tTime, 60) * 3.0 * ... * 10)
        let base = t_time.max(60.0);
        let factor = 3.0
            * opts.line_unmatched_punish_fact
            * opts.line_name_to_unmatched_punish_fact
            * opts.line_name_from_unmatched_punish_fact;

        (base * factor * 10.0).ceil() as u32
    }

    fn allows_fast_route() -> bool {
        true
    }

    fn need_dist() -> bool {
        false
    }
}

// TODO: Implement NormDistrTransWeight if needed
// TODO: Implement DistDiffTransWeight if needed
