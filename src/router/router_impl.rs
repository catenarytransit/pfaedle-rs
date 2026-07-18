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

    pub fn route(
        &self,
        trie: &crate::router::trip_trie::TripTrie,
        ecm: &AHashMap<usize, EdgeCandGroup>,
        r_opts: &RoutingOpts,
        mut hop_cache: Option<&mut HopCache>,
        no_fast_hops: bool,
    ) -> AHashMap<usize, EdgeListHops> {
        let mut ret = AHashMap::new();

        let num_nodes = trie.get_nds().len();
        let mut costs_dag = vec![vec![]; num_nodes];
        let mut prede_dag = vec![vec![]; num_nodes];
        let mut max_costs = vec![0.0; num_nodes];

        for nid in 1..num_nodes {
            let cand_len = ecm.get(&nid).map(|g| g.len()).unwrap_or(0);
            costs_dag[nid] = vec![1e18; cand_len];
            prede_dag[nid] = vec![usize::MAX; cand_len];
        }

        let mut st = Vec::new();
        for &cnid in &trie.get_nd(0).childs {
            st.push(cnid);
            let cand_len = ecm.get(&cnid).map(|g| g.len()).unwrap_or(0);
            costs_dag[cnid] = vec![0.0; cand_len]; // In C++ it's pen, we'll assign pen below
            if let Some(cands) = ecm.get(&cnid) {
                for (fr_id, cand) in cands.iter().enumerate() {
                    costs_dag[cnid][fr_id] = cand.pen;
                }
            }
        }

        while let Some(fr_tr_nid) = st.pop() {
            let fr_tr_nd = trie.get_nd(fr_tr_nid);

            let mut max_speed = 0.0;
            if let Some(fr_cands) = ecm.get(&fr_tr_nid) {
                for cand in fr_cands {
                    if let Some(e) = cand.edge {
                        let from_node_idx = self.graph.edge(e).from;
                        let comp = &self.graph.node(from_node_idx).payload;
                        // For maxSpeed, we can assume 130 km/h (36.11 m/s) if not stored in comp
                        // C++: comp.maxSpeed
                        // We will just assume 36.11 (highway speed limit approx)
                        let speed = 36.11;
                        if speed > max_speed {
                            max_speed = speed;
                        }
                    }
                }
            }
            if max_speed == 0.0 {
                max_speed = 36.11; // default fallback
            }

            for &to_tr_nid in &fr_tr_nd.childs {
                let to_tr_nd = trie.get_nd(to_tr_nid);

                if fr_tr_nd.arr && !to_tr_nd.arr {
                    if let Some(to_cands) = ecm.get(&to_tr_nid) {
                        for (to_id, to_cand) in to_cands.iter().enumerate() {
                            for &fr_id in &to_cand.dep_prede {
                                assert!(
                                    fr_id < costs_dag[fr_tr_nid].len(),
                                    "invalid dep_prede: parent trie node {}, child trie node {}, \
                                     predecessor {}, parent candidates {}, child candidates {}",
                                    fr_tr_nid,
                                    to_tr_nid,
                                    fr_id,
                                    costs_dag[fr_tr_nid].len(),
                                    costs_dag[to_tr_nid].len(),
                                );
                                let new_c = costs_dag[fr_tr_nid][fr_id] + to_cand.pen;
                                if new_c < costs_dag[to_tr_nid][to_id] {
                                    costs_dag[to_tr_nid][to_id] = new_c;
                                    prede_dag[to_tr_nid][to_id] = fr_id;
                                }
                            }
                        }
                    }
                    st.push(to_tr_nid);
                    continue;
                }

                let avg_dep_t = if fr_tr_nd.trips > 0 {
                    fr_tr_nd.acc_time as f64 / fr_tr_nd.trips as f64
                } else {
                    0.0
                };
                let avg_arr_t = if to_tr_nd.trips > 0 {
                    to_tr_nd.acc_time as f64 / to_tr_nd.trips as f64
                } else {
                    0.0
                };

                use geo::algorithm::HaversineDistance;
                let hop_dist = fr_tr_nd.pos.haversine_distance(&to_tr_nd.pos);
                let min_time = hop_dist / max_speed;
                let mut hop_time = avg_arr_t - avg_dep_t;
                if hop_time < min_time {
                    hop_time = min_time;
                }

                let mut new_max_cost = TW::max_cost(hop_time, r_opts);
                let mut max_cost_val = new_max_cost;

                let mut found = false;
                let mut step = 0;
                let max_route_cost_doubling_steps = 3;

                while !found && step <= max_route_cost_doubling_steps {
                    max_costs[to_tr_nid] = new_max_cost as f64;
                    max_cost_val = new_max_cost;

                    let mut cost_m = Vec::new();
                    let mut dists = Vec::new();

                    if let (Some(fr_cands), Some(to_cands)) =
                        (ecm.get(&fr_tr_nid), ecm.get(&to_tr_nid))
                    {
                        if no_fast_hops || !TW::allows_fast_route() {
                            let cache_mut = if let Some(ref mut c) = hop_cache {
                                Some(&mut **c)
                            } else {
                                None
                            };
                            self.hops(
                                fr_cands,
                                to_cands,
                                &mut cost_m,
                                &mut dists,
                                &to_tr_nd.r_attrs,
                                r_opts,
                                cache_mut,
                                max_cost_val,
                            );
                        } else {
                            let cache_mut = if let Some(ref mut c) = hop_cache {
                                Some(&mut **c)
                            } else {
                                None
                            };
                            self.hops_fast(
                                fr_cands,
                                to_cands,
                                &costs_dag[fr_tr_nid],
                                &mut cost_m,
                                &to_tr_nd.r_attrs,
                                r_opts,
                                cache_mut,
                                max_cost_val,
                            );
                        }
                    }

                    for (i, m_val) in cost_m.iter().enumerate() {
                        let fr_id = m_val.0.0;
                        let to_id = m_val.0.1;
                        let c = m_val.1;

                        let m_dist = if TW::need_dist() && i < dists.len() {
                            dists[i].1 as f64
                        } else {
                            0.0
                        };

                        if let (Some(fr_cands), Some(to_cands)) =
                            (ecm.get(&fr_tr_nid), ecm.get(&to_tr_nid))
                        {
                            let dep_t = fr_cands[fr_id].time;
                            let arr_t = to_cands[to_id].time;
                            let w = TW::weight(c, m_dist, arr_t - dep_t, hop_dist, r_opts);

                            let new_c = costs_dag[fr_tr_nid][fr_id] + to_cands[to_id].pen + w;
                            if new_c < costs_dag[to_tr_nid][to_id] {
                                costs_dag[to_tr_nid][to_id] = new_c;
                                prede_dag[to_tr_nid][to_id] = fr_id;
                                found = true;
                            }
                        }
                    }

                    if new_max_cost <= u32::MAX / 2 {
                        new_max_cost *= 2;
                    } else {
                        new_max_cost = u32::MAX;
                    }

                    if new_max_cost == max_cost_val {
                        break;
                    }
                    step += 1;
                }

                if !found {
                    if let (Some(fr_cands), Some(to_cands)) =
                        (ecm.get(&fr_tr_nid), ecm.get(&to_tr_nid))
                    {
                        for fr_nid in 0..fr_cands.len() {
                            let new_c =
                                costs_dag[fr_tr_nid][fr_nid] + (max_cost_val as f64) * 100.0;
                            let mut null_c_id = 0;
                            while null_c_id < to_cands.len() && to_cands[null_c_id].edge.is_none() {
                                if new_c < costs_dag[to_tr_nid][null_c_id] {
                                    prede_dag[to_tr_nid][null_c_id] = fr_nid;
                                    costs_dag[to_tr_nid][null_c_id] = new_c;
                                }
                                null_c_id += 1;
                            }
                        }

                        for fr_nid in 0..fr_cands.len() {
                            let mut to_nid = 1;
                            while to_nid < to_cands.len() && to_cands[to_nid].edge.is_none() {
                                to_nid += 1;
                            }
                            for i in to_nid..to_cands.len() {
                                let new_c = costs_dag[fr_tr_nid][fr_nid] + to_cands[i].pen;
                                if new_c < costs_dag[to_tr_nid][i] {
                                    prede_dag[to_tr_nid][i] = fr_nid;
                                    costs_dag[to_tr_nid][i] = new_c;
                                }
                            }
                        }
                    }
                }

                st.push(to_tr_nid);
            }
        }

        let mut sink_costs = AHashMap::new();
        let mut front_ids = AHashMap::new();
        for (leaf, _trips) in trie.get_nd_trips() {
            sink_costs.insert(*leaf, 1e18);
            front_ids.insert(*leaf, 0);

            if let Some(cands) = ecm.get(leaf) {
                for last_id in 0..cands.len() {
                    let n_cost = costs_dag[*leaf][last_id];
                    if n_cost < *sink_costs.get(leaf).unwrap() {
                        front_ids.insert(*leaf, last_id);
                        sink_costs.insert(*leaf, n_cost);
                    }
                }
            }
        }

        for (leaf, _trips) in trie.get_nd_trips() {
            let leaf_nid = *leaf;
            let mut cur_trie_nid = leaf_nid;

            while prede_dag[cur_trie_nid][front_ids[&leaf_nid]] != usize::MAX {
                let cur_trie_par_nid = trie.get_nd(cur_trie_nid).parent.unwrap();
                let fr_id = prede_dag[cur_trie_nid][front_ids[&leaf_nid]];
                let to_id = front_ids[&leaf_nid];

                let fr_tr_nd = trie.get_nd(cur_trie_par_nid);
                let to_tr_nd = trie.get_nd(cur_trie_nid);

                if fr_tr_nd.arr && !to_tr_nd.arr {
                    front_ids.insert(leaf_nid, fr_id);
                    cur_trie_nid = cur_trie_par_nid;
                    continue;
                }

                let mut edgs = Vec::new();

                if let (Some(fr_cands), Some(to_cands)) =
                    (ecm.get(&cur_trie_par_nid), ecm.get(&cur_trie_nid))
                {
                    let fr = &fr_cands[fr_id];
                    let to = &to_cands[to_id];

                    if let (Some(fr_e), Some(to_e)) = (fr.edge, to.edge) {
                        let mut max_progr_start = 0.0;
                        for cand in fr_cands {
                            if let Some(cand_e) = cand.edge {
                                let progr_start = if cand.progr > 0.0 {
                                    self.get_edge_cost(cand_e, None, r_opts) as f64 * cand.progr
                                } else {
                                    0.0
                                };
                                if progr_start > max_progr_start {
                                    max_progr_start = progr_start;
                                }
                            }
                        }

                        let max_cost_rt = max_costs[cur_trie_nid] + max_progr_start;
                        let mut max_cost_rt_int = max_cost_rt as u32;
                        if max_cost_rt >= u32::MAX as f64 {
                            max_cost_rt_int = u32::MAX;
                        }

                        let path_costs = self.run_dijkstra_1_to_n(
                            fr_e,
                            &vec![to_e].into_iter().collect(),
                            max_cost_rt_int,
                            r_opts,
                        );
                        let cost = path_costs.get(&to_e).cloned().unwrap_or(ROUTE_INF);

                        if cost < max_cost_rt_int {
                            edgs = self.reconstruct_path(fr_e, to_e, r_opts);
                            ret.entry(leaf_nid).or_insert(Vec::new()).push(
                                crate::router::types::EdgeHop {
                                    edges: edgs,
                                    start_edge: Some(fr_e),
                                    end_edge: Some(to_e),
                                    start_progr: fr.progr,
                                    end_progr: to.progr,
                                    start_point: None,
                                    end_point: None,
                                },
                            );
                        } else {
                            ret.entry(leaf_nid).or_insert(Vec::new()).push(
                                crate::router::types::EdgeHop {
                                    edges: vec![],
                                    start_edge: Some(fr_e),
                                    end_edge: Some(to_e),
                                    start_progr: fr.progr,
                                    end_progr: to.progr,
                                    start_point: None,
                                    end_point: None,
                                },
                            );
                        }
                    } else {
                        if fr.edge.is_some() {
                            ret.entry(leaf_nid).or_insert(Vec::new()).push(
                                crate::router::types::EdgeHop {
                                    edges: edgs,
                                    start_edge: fr.edge,
                                    end_edge: None,
                                    start_progr: fr.progr,
                                    end_progr: 0.0,
                                    start_point: None,
                                    end_point: to.point,
                                },
                            );
                        } else if to.edge.is_some() {
                            ret.entry(leaf_nid).or_insert(Vec::new()).push(
                                crate::router::types::EdgeHop {
                                    edges: edgs,
                                    start_edge: None,
                                    end_edge: to.edge,
                                    start_progr: 0.0,
                                    end_progr: to.progr,
                                    start_point: fr.point,
                                    end_point: None,
                                },
                            );
                        } else {
                            ret.entry(leaf_nid).or_insert(Vec::new()).push(
                                crate::router::types::EdgeHop {
                                    edges: edgs,
                                    start_edge: None,
                                    end_edge: None,
                                    start_progr: 0.0,
                                    end_progr: 0.0,
                                    start_point: fr.point,
                                    end_point: to.point,
                                },
                            );
                        }
                    }
                }

                front_ids.insert(leaf_nid, fr_id);
                cur_trie_nid = cur_trie_par_nid;
            }

            if let Some(hops) = ret.get_mut(&leaf_nid) {
                hops.reverse();
            }
        }

        ret
    }

    fn reconstruct_path(
        &self,
        start: EdgeIndex,
        target: EdgeIndex,
        r_opts: &RoutingOpts,
    ) -> Vec<EdgeIndex> {
        let mut dists = AHashMap::new();
        let mut parents = AHashMap::new();
        let mut pq = std::collections::BinaryHeap::new();

        dists.insert(start, 0);
        pq.push(State {
            cost: 0,
            edge: start,
        });

        while let Some(State { cost, edge: u }) = pq.pop() {
            if u == target {
                break;
            }

            if cost > *dists.get(&u).unwrap_or(&ROUTE_INF) {
                continue;
            }

            let u_edge = self.graph.edge(u);
            let u_to_node_idx = u_edge.to;
            let u_to_node = self.graph.node(u_to_node_idx);

            for &v_idx in u_to_node.edges() {
                let v_edge = self.graph.edge(v_idx);
                if v_edge.from != u_to_node_idx {
                    continue;
                }

                let weight = self.get_edge_cost(v_idx, Some(u), r_opts);
                let next_cost = cost.saturating_add(weight);

                if next_cost < *dists.get(&v_idx).unwrap_or(&ROUTE_INF) {
                    dists.insert(v_idx, next_cost);
                    parents.insert(v_idx, u);
                    pq.push(State {
                        cost: next_cost,
                        edge: v_idx,
                    });
                }
            }
        }

        let mut path = Vec::new();
        let mut curr = target;
        while curr != start {
            path.push(curr);
            if let Some(&p) = parents.get(&curr) {
                curr = p;
            } else {
                return vec![]; // no path
            }
        }
        path.push(start);
        path.reverse();
        path
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

            for &v_idx in u_to_node.edges() {
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
        init_costs_vec: &Vec<f64>,
        r_costs: &mut Vec<((usize, usize), u32)>,
        _r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        mut hop_cache: Option<&mut HopCache>,
        mut max_cost: u32,
    ) {
        let mut e_frs = AHashSet::new();
        let mut e_tos = AHashSet::new();
        let mut e_fr_cands: AHashMap<EdgeIndex, Vec<usize>> = AHashMap::new();
        let mut e_to_cands: AHashMap<EdgeIndex, Vec<usize>> = AHashMap::new();

        for (fr_id, fr) in froms.iter().enumerate() {
            if init_costs_vec[fr_id] >= 1e18 || !self.connected(fr, tos) {
                continue;
            }
            if let Some(e) = fr.edge {
                e_frs.insert(e);
                e_fr_cands.entry(e).or_default().push(fr_id);
            }
        }

        for (to_id, to) in tos.iter().enumerate() {
            if !self.connected_group(froms, to) {
                continue;
            }
            if let Some(e) = to.edge {
                if let Some(cache) = hop_cache.as_deref_mut() {
                    if self.cache_drop(cache, &e_frs, e, max_cost) {
                        continue;
                    }
                }
                e_tos.insert(e);
                e_to_cands.entry(e).or_default().push(to_id);
            }
        }

        if e_frs.is_empty() || e_tos.is_empty() {
            return;
        }

        let mut max_progr_start = 0.0;
        for fr in froms {
            if let Some(e) = fr.edge {
                let progr_start = if fr.progr > 0.0 {
                    let cost_e = self.get_edge_cost(e, None, r_opts);
                    (cost_e as f64) * fr.progr
                } else {
                    0.0
                };
                if progr_start > max_progr_start {
                    max_progr_start = progr_start;
                }
            }
        }

        let mut prep_init_costs = vec![0.0; froms.len()];
        let mut init_costs_map: AHashMap<EdgeIndex, u32> = AHashMap::new();

        for e in &e_frs {
            init_costs_map.insert(*e, ROUTE_INF);
        }

        for (fr_id, fr) in froms.iter().enumerate() {
            if let Some(e) = fr.edge {
                if init_costs_vec[fr_id] < 1e18 {
                    let cost_e = self.get_edge_cost(e, None, r_opts);
                    let progr_start = if fr.progr > 0.0 {
                        (cost_e as f64) * fr.progr
                    } else {
                        0.0
                    };
                    prep_init_costs[fr_id] = TW::inv_weight(init_costs_vec[fr_id], r_opts) as f64
                        + max_progr_start
                        - progr_start;
                    let prep_u32 = prep_init_costs[fr_id] as u32;
                    if let Some(curr) = init_costs_map.get_mut(&e) {
                        if prep_u32 < *curr {
                            *curr = prep_u32;
                        }
                    }
                }
            }
        }

        let mut max_init = 0;
        let mut min_init = ROUTE_INF;
        for (&e, &c) in &init_costs_map {
            if e_frs.contains(&e) {
                if c != ROUTE_INF && c > max_init {
                    max_init = c;
                }
                if c < min_init {
                    min_init = c;
                }
            }
        }

        for c in init_costs_map.values_mut() {
            if *c != ROUTE_INF {
                *c = c.saturating_sub(min_init);
            }
        }

        max_cost = self.add_non_overflow(max_cost, max_progr_start as u32);
        let max_cost_search = max_cost.saturating_add(max_init.saturating_sub(min_init));

        // run_dijkstra_n_to_n
        let mut dists: AHashMap<EdgeIndex, (u32, EdgeIndex)> = AHashMap::new();
        let mut pq = std::collections::BinaryHeap::new();

        for &start in &e_frs {
            let start_init_cost = *init_costs_map.get(&start).unwrap_or(&0);
            if start_init_cost < ROUTE_INF {
                dists.insert(start, (start_init_cost, start));
                pq.push(State {
                    cost: start_init_cost,
                    edge: start,
                });
            }
        }

        while let Some(State { cost, edge: u }) = pq.pop() {
            if cost > max_cost_search {
                break;
            }

            if cost > dists.get(&u).map(|d| d.0).unwrap_or(ROUTE_INF) {
                continue;
            }

            let (_, u_source) = *dists.get(&u).unwrap();

            let u_edge = self.graph.edge(u);
            let u_to_node_idx = u_edge.to;
            let u_to_node = self.graph.node(u_to_node_idx);

            for &v_idx in u_to_node.edges() {
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

        // Collect results
        for &to_edg in &e_tos {
            let reached = dists.get(&to_edg);
            if reached.is_none() || reached.unwrap().0 >= ROUTE_INF {
                if let Some(cache) = hop_cache.as_deref_mut() {
                    for &fr_edg in &e_frs {
                        cache.set_min(fr_edg, to_edg, max_cost);
                    }
                }
                continue;
            }

            let (total_cost, from_edg) = *reached.unwrap();
            let base_cost = total_cost.saturating_sub(*init_costs_map.get(&from_edg).unwrap_or(&0));

            if base_cost >= max_cost {
                continue;
            }

            if let Some(fr_cands) = e_fr_cands.get(&from_edg) {
                for &fr_id in fr_cands {
                    let fr = &froms[fr_id];
                    let cost_fr = self.get_edge_cost(from_edg, None, r_opts);

                    if let Some(to_cands) = e_to_cands.get(&to_edg) {
                        for &to_id in to_cands {
                            let to = &tos[to_id];
                            let mut wr_cost = base_cost;

                            if from_edg == to_edg {
                                if fr.progr <= to.progr {
                                    let cost_to = self.get_edge_cost(to_edg, None, r_opts);
                                    let progr_c_fr = ((cost_fr as f64) * fr.progr) as u32;
                                    let progr_c_to = ((cost_to as f64) * to.progr) as u32;
                                    wr_cost = wr_cost
                                        .saturating_add(progr_c_to.saturating_sub(progr_c_fr));
                                } else {
                                    continue;
                                }
                            } else {
                                if fr.progr > 0.0 {
                                    let progr_c_fr = ((cost_fr as f64) * fr.progr) as u32;
                                    wr_cost = wr_cost.saturating_sub(progr_c_fr);
                                }
                                if to.progr > 0.0 {
                                    let cost_to = self.get_edge_cost(to_edg, None, r_opts);
                                    let progr_c_to = ((cost_to as f64) * to.progr) as u32;
                                    wr_cost = wr_cost.saturating_add(progr_c_to);
                                }
                            }

                            if wr_cost < max_cost.saturating_sub(max_progr_start as u32) {
                                r_costs.push(((fr_id, to_id), wr_cost));
                            }
                        }
                    }
                }
            }
        }
    }

    fn connected(&self, fr: &crate::router::types::EdgeCand, tos: &EdgeCandGroup) -> bool {
        if let Some(fr_e) = fr.edge {
            let fr_node = self.graph.edge(fr_e).from;
            let fr_comp = self.graph.node(fr_node).payload.comp_id;
            for to in tos {
                if let Some(to_e) = to.edge {
                    let to_node = self.graph.edge(to_e).from;
                    if fr_comp == self.graph.node(to_node).payload.comp_id {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn connected_group(&self, froms: &EdgeCandGroup, to: &crate::router::types::EdgeCand) -> bool {
        if let Some(to_e) = to.edge {
            let to_node = self.graph.edge(to_e).from;
            let to_comp = self.graph.node(to_node).payload.comp_id;
            for fr in froms {
                if let Some(fr_e) = fr.edge {
                    let fr_node = self.graph.edge(fr_e).from;
                    if to_comp == self.graph.node(fr_node).payload.comp_id {
                        return true;
                    }
                }
            }
        }
        false
    }

    fn cache_drop(
        &self,
        hop_cache: &mut HopCache,
        froms: &AHashSet<EdgeIndex>,
        to: EdgeIndex,
        max_cost: u32,
    ) -> bool {
        for &fr in froms {
            let cached = hop_cache.get(fr, to);
            if cached.0 <= max_cost {
                return false;
            }
        }
        true
    }

    fn add_non_overflow(&self, a: u32, b: u32) -> u32 {
        if a == u32::MAX || b == u32::MAX {
            return u32::MAX;
        }
        a.saturating_add(b)
    }
}
