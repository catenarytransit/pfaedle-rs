use crate::graph::{EdgeIndex, EdgePL, Graph, NodePL};
use crate::router::hop_cache::HopCache;
use crate::router::types::{EdgeCandGroup, EdgeListHops};
use crate::router::weights::{RoutingAttrs, RoutingOpts, TransWeight};
use ahash::{AHashMap, AHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::marker::PhantomData;

const ROUTE_INF: u32 = 2_000_000_000;
// const DBL_INF: f64 = 1e18;

pub struct RouterImpl<'a, TW: TransWeight> {
    graph: &'a Graph<NodePL, EdgePL>,
    _phantom: PhantomData<TW>,
}

#[derive(Clone, Copy, PartialEq)]
struct State {
    cost: u32,
    edge: EdgeIndex,
}

impl Eq for State {}

impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}

impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<'a, TW: TransWeight> RouterImpl<'a, TW> {
    pub fn new(graph: &'a Graph<NodePL, EdgePL>) -> Self {
        Self {
            graph,
            _phantom: PhantomData,
        }
    }

    // Placeholder for route
    pub fn route(
        &self,
        _trie: &crate::router::trip_trie::TripTrie, // Add imports if needed
        _ecm: &AHashMap<EdgeIndex, Vec<usize>>,
        _r_opts: &RoutingOpts,
        _hop_cache: Option<&mut HopCache>,
        _no_fast_hops: bool,
    ) -> AHashMap<usize, EdgeListHops> {
        AHashMap::new()
    }

    fn get_edge_cost(
        &self,
        edge_idx: EdgeIndex,
        _prev_edge: Option<EdgeIndex>,
        _r_opts: &RoutingOpts,
    ) -> u32 {
        let edge = self.graph.edge(edge_idx);
        edge.payload.cost
    }

    fn hops(
        &self,
        froms: &EdgeCandGroup,
        tos: &EdgeCandGroup,
        r_costs: &mut Vec<((usize, usize), u32)>,
        _dists: &mut Vec<((usize, usize), u32)>,
        _r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        mut hop_cache: Option<&mut HopCache>,
        max_cost: u32,
    ) {
        let mut e_frs = AHashSet::new();
        for fr in froms {
            if let Some(e) = fr.edge {
                e_frs.insert(e);
            }
        }

        let mut e_tos = AHashSet::new();
        for to in tos {
            if let Some(e) = to.edge {
                e_tos.insert(e);
            }
        }

        let mut ecm_cost: AHashMap<(EdgeIndex, EdgeIndex), u32> = AHashMap::new();

        for &e_from in &e_frs {
            let mut rem_tos = AHashSet::new();

            for &e_to in &e_tos {
                let cached = if let Some(cache) = &hop_cache {
                    cache.get(e_from, e_to)
                } else {
                    (0, false)
                };

                if cached.1 && cached.0 >= ROUTE_INF {
                    ecm_cost.insert((e_from, e_to), ROUTE_INF);
                } else if !TW::need_dist() && cached.1 {
                    ecm_cost.insert((e_from, e_to), cached.0);
                } else {
                    rem_tos.insert(e_to);
                }
            }

            if !rem_tos.is_empty() {
                let costs = self.run_dijkstra_1_to_n(e_from, &rem_tos, max_cost, r_opts);

                for (to_e, cost) in costs {
                    ecm_cost.insert((e_from, to_e), cost);
                    if let Some(cache) = hop_cache.as_deref_mut() {
                        if cost == ROUTE_INF {
                            cache.set_min(e_from, to_e, max_cost);
                        } else {
                            cache.set_ex(e_from, to_e, cost);
                        }
                    }
                }
            }
        }

        for (fr_id, fr) in froms.iter().enumerate() {
            let e_fr = match fr.edge {
                Some(e) => e,
                None => continue,
            };

            let cost_fr = self.get_edge_cost(e_fr, None, r_opts);

            for (to_id, to) in tos.iter().enumerate() {
                let e_to = match to.edge {
                    Some(e) => e,
                    None => continue,
                };

                // Retrieve base cost
                let val = ecm_cost.get(&(e_fr, e_to)).cloned().unwrap_or(ROUTE_INF);
                if val >= max_cost {
                    continue;
                }

                let mut c = val;

                if e_fr == e_to {
                    if fr.progr <= to.progr {
                        let cost_to = self.get_edge_cost(e_to, None, r_opts);
                        let progr_c_fr = (cost_fr as f64 * fr.progr) as u32;
                        let progr_c_to = (cost_to as f64 * to.progr) as u32;
                        c += progr_c_to - progr_c_fr;
                    } else {
                        continue;
                    }
                } else {
                    if fr.progr > 0.0 {
                        let progr_c_fr = (cost_fr as f64 * fr.progr) as u32;
                        c = c.saturating_sub(progr_c_fr);
                    }
                    if to.progr > 0.0 {
                        let cost_to = self.get_edge_cost(e_to, None, r_opts);
                        let progr_c_to = (cost_to as f64 * to.progr) as u32;
                        c = c.saturating_add(progr_c_to);
                    }
                }

                if c < max_cost {
                    r_costs.push(((fr_id, to_id), c));
                }
            }
        }
    }

    fn run_dijkstra_1_to_n(
        &self,
        start: EdgeIndex,
        targets: &AHashSet<EdgeIndex>,
        max_cost: u32,
        r_opts: &RoutingOpts,
    ) -> AHashMap<EdgeIndex, u32> {
        let mut dists = AHashMap::new();
        let mut pq = BinaryHeap::new();

        dists.insert(start, 0);
        pq.push(State {
            cost: 0,
            edge: start,
        });

        while let Some(State { cost, edge: u }) = pq.pop() {
            if cost > max_cost {
                break;
            }

            if cost > *dists.get(&u).unwrap_or(&ROUTE_INF) {
                continue;
            }

            let u_edge = self.graph.edge(u);
            let u_to_node_idx = u_edge.to;
            let u_to_node = self.graph.node(u_to_node_idx);

            for &v_idx in &u_to_node.edges {
                let v_edge = self.graph.edge(v_idx);
                if v_edge.from != u_to_node_idx {
                    continue;
                }

                let weight = self.get_edge_cost(v_idx, Some(u), r_opts);
                let next_cost = cost.saturating_add(weight);

                if next_cost < *dists.get(&v_idx).unwrap_or(&ROUTE_INF) {
                    dists.insert(v_idx, next_cost);
                    pq.push(State {
                        cost: next_cost,
                        edge: v_idx,
                    });
                }
            }
        }

        let mut res = AHashMap::new();
        for &t in targets {
            if let Some(&c) = dists.get(&t) {
                res.insert(t, c);
            } else {
                res.insert(t, ROUTE_INF);
            }
        }
        res
    }

    fn hops_fast(
        &self,
        froms: &EdgeCandGroup,
        tos: &EdgeCandGroup,
        init_costs: &Vec<f64>,
        _r_costs: &mut Vec<((usize, usize), u32)>,
        _r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        _hop_cache: Option<&mut HopCache>,
        max_cost: u32,
    ) {
        let mut dists: AHashMap<EdgeIndex, (u32, EdgeIndex)> = AHashMap::new(); // Edge -> (Cost, SourceEdge)
        let mut pq = BinaryHeap::new();

        let mut e_fr_cands: AHashMap<EdgeIndex, Vec<usize>> = AHashMap::new();
        let mut e_to_cands: AHashMap<EdgeIndex, Vec<usize>> = AHashMap::new();

        for (fr_id, fr) in froms.iter().enumerate() {
            if let Some(e) = fr.edge {
                if init_costs[fr_id] >= 1e18 {
                    continue;
                }

                let w = TW::inv_weight(init_costs[fr_id], r_opts);
                e_fr_cands.entry(e).or_default().push(fr_id);

                if w < dists.get(&e).map(|d| d.0).unwrap_or(ROUTE_INF) {
                    dists.insert(e, (w, e));
                    pq.push(State { cost: w, edge: e });
                }
            }
        }

        for (to_id, to) in tos.iter().enumerate() {
            if let Some(e) = to.edge {
                e_to_cands.entry(e).or_default().push(to_id);
            }
        }

        while let Some(State { cost, edge: u }) = pq.pop() {
            if cost > max_cost + 100000 {
                break;
            }

            if cost > dists.get(&u).map(|d| d.0).unwrap_or(ROUTE_INF) {
                continue;
            }

            // Retrieve source for reconstruction
            let (_, u_source) = *dists.get(&u).unwrap();

            if let Some(_cands) = e_to_cands.get(&u) {
                // Logic to reconstruct paths
            }

            let u_edge = self.graph.edge(u);
            let u_to_node_idx = u_edge.to;
            let u_to_node = self.graph.node(u_to_node_idx);

            for &v_idx in &u_to_node.edges {
                let v_edge = self.graph.edge(v_idx);
                if v_edge.from != u_to_node_idx {
                    continue;
                }

                let weight = self.get_edge_cost(v_idx, Some(u), r_opts);
                let next_cost = cost.saturating_add(weight);

                if next_cost < dists.get(&v_idx).map(|d| d.0).unwrap_or(ROUTE_INF) {
                    dists.insert(v_idx, (next_cost, u_source));
                    pq.push(State {
                        cost: next_cost,
                        edge: v_idx,
                    });
                }
            }
        }
    }
}
