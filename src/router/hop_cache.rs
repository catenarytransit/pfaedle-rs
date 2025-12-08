use crate::graph::EdgeIndex;
use ahash::{AHashMap, AHashSet};

pub struct HopCache {
    // Maps (from_edge, to_edge) -> value
    // Value encoding (same as C++):
    // >= 0: Min value (not exact)
    // < 0: Exact value = (-v) - 1
    cache: AHashMap<(EdgeIndex, EdgeIndex), i64>,
}

impl HopCache {
    pub fn new() -> Self {
        Self {
            cache: AHashMap::new(),
        }
    }

    pub fn set_min(&mut self, a: EdgeIndex, b: EdgeIndex, val: u32) {
        // C++: _cache.set(a, b, val);
        // Assuming underlying set just overwrites or possibly checks?
        // In C++ source provided: _cache.set(a, b, val) implies direct set.
        // However, usually caches might keep the "better" value.
        // Let's assume overwrite for now as per the snippet simple forwarding.
        self.cache.insert((a, b), val as i64);
    }

    pub fn set_ex(&mut self, a: EdgeIndex, b: EdgeIndex, val: u32) {
        // C++: int64_t v = val; _cache.set(a, b, -(v + 1));
        let v = val as i64;
        self.cache.insert((a, b), -(v + 1));
    }

    pub fn set_min_set(&mut self, a: EdgeIndex, b: &AHashSet<EdgeIndex>, val: u32) {
        for &eb in b {
            self.set_min(a, eb, val);
        }
    }

    pub fn set_min_from_set(&mut self, a: &AHashSet<EdgeIndex>, b: EdgeIndex, val: u32) {
        for &ea in a {
            self.set_min(ea, b, val);
        }
    }

    // Returns (value, is_exact)
    pub fn get(&self, a: EdgeIndex, b: EdgeIndex) -> (u32, bool) {
        match self.cache.get(&(a, b)) {
            Some(&v) => {
                if v < 0 {
                    // C++: return {(-v) - 1, 1};
                    ((-v - 1) as u32, true)
                } else {
                    // C++: return {v, 0};
                    (v as u32, false)
                }
            }
            None => (0, false), // Or some default/sentinel if not found?
        }
    }
}
