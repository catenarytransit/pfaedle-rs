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

/// Cheap line-name comparison used on every explored routing edge.
///
/// Upstream pfaedle accepts exact matches plus whole-token prefix/suffix
/// matches. We also compare ASCII-alphanumeric streams so common GTFS/OSM
/// spelling differences such as `RE3` versus `RE 3` do not require allocating
/// normalized Strings in the Dijkstra hot path.
fn line_simi(a: &str, b: &str) -> bool {
    if a.eq_ignore_ascii_case(b) {
        return true;
    }
    if a.is_empty() || b.is_empty() {
        return false;
    }

    fn compact_eq(a: &str, b: &str) -> bool {
        // Do not let two punctuation-only strings compare equal after filtering.
        if !a.bytes().any(|byte| byte.is_ascii_alphanumeric())
            || !b.bytes().any(|byte| byte.is_ascii_alphanumeric())
        {
            return false;
        }

        a.bytes()
            .filter(|byte| byte.is_ascii_alphanumeric())
            .map(|byte| byte.to_ascii_lowercase())
            .eq(b
                .bytes()
                .filter(|byte| byte.is_ascii_alphanumeric())
                .map(|byte| byte.to_ascii_lowercase()))
    }

    fn begins_with_word(longer: &str, shorter: &str) -> bool {
        if longer.len() <= shorter.len() + 1 || !longer.is_char_boundary(shorter.len()) {
            return false;
        }
        longer[..shorter.len()].eq_ignore_ascii_case(shorter)
            && longer[shorter.len()..]
                .chars()
                .next()
                .is_some_and(char::is_whitespace)
    }

    fn ends_with_word(longer: &str, shorter: &str) -> bool {
        if longer.len() <= shorter.len() + 1 || longer.len() < shorter.len() {
            return false;
        }
        let start = longer.len() - shorter.len();
        if !longer.is_char_boundary(start) || start == 0 {
            return false;
        }
        longer[start..].eq_ignore_ascii_case(shorter)
            && longer[..start]
                .chars()
                .next_back()
                .is_some_and(char::is_whitespace)
    }

    compact_eq(a, b)
        || begins_with_word(a, b)
        || ends_with_word(a, b)
        || begins_with_word(b, a)
        || ends_with_word(b, a)
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
        // Match upstream's shortcut: an edge with no line information should not
        // be punished merely because its metadata is absent.
        if info.short_name.is_empty() && info.from_str.is_empty() && info.to_str.is_empty() {
            return LineSimilarity {
                name_similar: true,
                from_similar: true,
                to_similar: true,
            };
        }

        LineSimilarity {
            name_similar: self.short_name.is_empty()
                || line_simi(&info.short_name, &self.short_name),
            from_similar: self.line_from.is_empty()
                || info.from_str.eq_ignore_ascii_case(&self.line_from),
            to_similar: self.line_to.is_empty() || info.to_str.eq_ignore_ascii_case(&self.line_to),
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

#[cfg(test)]
mod tests {
    use super::line_simi;

    #[test]
    fn line_similarity_is_token_aware() {
        assert!(line_simi("Red Line", "Red"));
        assert!(line_simi("RE 3", "RE3"));
        assert!(line_simi("re3", "RE 3"));
        assert!(!line_simi("62", "162"));
        assert!(!line_simi("RE3", "RE55"));
        assert!(!line_simi("---", "___"));
    }
}
