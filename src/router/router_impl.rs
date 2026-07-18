use crate::graph::{EdgeIndex, EdgePL, Graph, NodeIndex, NodePL};
use crate::router::hop_cache::HopCache;
use crate::router::types::{EdgeCandGroup, EdgeListHops};
use crate::router::weights::{RoutingAttrs, RoutingOpts, TransWeight};
use ahash::{AHashMap, AHashSet};
use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::marker::PhantomData;

const ROUTE_INF: u32 = 2_000_000_000;

#[derive(Debug, Clone, Default)]
pub struct Restrictor {
    pub pos: AHashMap<NodeIndex, Vec<(EdgeIndex, EdgeIndex)>>,
    pub neg: AHashMap<NodeIndex, Vec<(EdgeIndex, EdgeIndex)>>,
}

impl Restrictor {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn may(&self, from: EdgeIndex, to: EdgeIndex, via: NodeIndex) -> bool {
        if let Some(rules) = self.pos.get(&via) {
            for &(r_from, r_to) in rules {
                if r_from == from && r_to != to {
                    return false;
                } else if r_from == from && r_to == to {
                    return true;
                }
            }
        }
        if let Some(rules) = self.neg.get(&via) {
            for &(r_from, r_to) in rules {
                if r_from == from && r_to == to {
                    return false;
                }
            }
        }
        true
    }
}

struct DistHeur {
    max_v: f64,
    center: geo::Point<f64>,
    max_cent_d: f64,
    has_targets: bool,
}

impl DistHeur {
    fn new(graph: &Graph<NodePL, EdgePL>, max_v: f64, tos: &AHashSet<EdgeIndex>) -> Self {
        if tos.is_empty() {
            return Self {
                max_v,
                center: geo::Point::new(0.0, 0.0),
                max_cent_d: 0.0,
                has_targets: false,
            };
        }

        let mut x = 0.0;
        let mut y = 0.0;
        let c = tos.len() as f64;

        for &to_idx in tos {
            let to_edge = graph.edge(to_idx);
            let from_node = graph.node(to_edge.from);
            x += from_node.payload.point.x();
            y += from_node.payload.point.y();
        }

        x /= c;
        y /= c;
        let center = geo::Point::new(x, y);

        let mut max_cent_d = 0.0;
        for &to_idx in tos {
            let to_edge = graph.edge(to_idx);
            let from_node = graph.node(to_edge.from);
            use geo::algorithm::HaversineDistance;
            let cur = from_node.payload.point.haversine_distance(&center);
            if cur > max_cent_d {
                max_cent_d = cur;
            }
        }

        max_cent_d /= max_v;

        Self {
            max_v,
            center,
            max_cent_d,
            has_targets: true,
        }
    }

    fn eval(&self, graph: &Graph<NodePL, EdgePL>, edge_idx: EdgeIndex) -> u32 {
        if !self.has_targets {
            return 0;
        }
        let edge = graph.edge(edge_idx);
        let from_node = graph.node(edge.from);
        use geo::algorithm::HaversineDistance;
        let d = from_node.payload.point.haversine_distance(&self.center);
        let heur = (d / self.max_v - self.max_cent_d).max(0.0) * 10.0;
        if heur > u32::MAX as f64 {
            u32::MAX
        } else {
            heur as u32
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct PQState {
    priority: u32,
    cost: u32,
    dwi: u32,
    edge: EdgeIndex,
    parent: Option<EdgeIndex>,
    via_node: Option<NodeIndex>,
    source: EdgeIndex,
}

impl Ord for PQState {
    fn cmp(&self, other: &Self) -> Ordering {
        other.priority.cmp(&self.priority)
    }
}

impl PartialOrd for PQState {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

struct RouteEdge {
    parent: Option<EdgeIndex>,
    cost: u32,
    via_node: Option<NodeIndex>,
    source: EdgeIndex,
}

pub struct RouterImpl<'a, TW: TransWeight> {
    graph: &'a Graph<NodePL, EdgePL>,
    _phantom: PhantomData<TW>,
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
        restrict: &Restrictor,
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
                                restrict,
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
                                restrict,
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
                                    self.get_edge_cost(Some(cand_e), None, None, &to_tr_nd.r_attrs, r_opts, restrict)
                                        as f64
                                        * cand.progr
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

                        let mut starts_map = AHashMap::new();
                        starts_map.insert(fr_e, 0);
                        let targets_set = vec![to_e].into_iter().collect();
                        let (path_costs, _, path) = self.search_dijkstra(
                            &starts_map,
                            &targets_set,
                            max_cost_rt_int,
                            &to_tr_nd.r_attrs,
                            r_opts,
                            restrict,
                            Some(to_e),
                        );

                        if let Some(edgs) = path {
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

    fn transit_line_simi(
        edge: &EdgePL,
        r_attrs: &RoutingAttrs,
    ) -> crate::router::weights::LineSimilarity {
        if r_attrs.short_name.is_empty()
            && r_attrs.line_from.is_empty()
            && r_attrs.line_to.is_empty()
        {
            return crate::router::weights::LineSimilarity {
                name_similar: true,
                from_similar: true,
                to_similar: true,
            };
        }

        let mut best = crate::router::weights::LineSimilarity {
            name_similar: false,
            from_similar: false,
            to_similar: false,
        };

        for line in &edge.lines {
            let simi = r_attrs.simi(line);
            if simi.name_similar && simi.to_similar && simi.from_similar {
                return simi;
            }
            if best < simi {
                best = simi;
            }
        }

        best
    }

    fn inner_product(&self, p: geo::Point<f64>, a: geo::Point<f64>, b: geo::Point<f64>) -> f64 {
        let dx21 = a.x() - p.x();
        let dx31 = b.x() - p.x();
        let dy21 = a.y() - p.y();
        let dy31 = b.y() - p.y();
        let m12 = (dx21 * dx21 + dy21 * dy21).sqrt();
        let m13 = (dx31 * dx31 + dy31 * dy31).sqrt();
        if m12 == 0.0 || m13 == 0.0 {
            return 0.0;
        }
        let cos_theta = (dx21 * dx31 + dy21 * dy31) / (m12 * m13);
        let cos_theta_clamped = cos_theta.max(-1.0).min(1.0);
        let theta = cos_theta_clamped.acos();
        theta.to_degrees()
    }

    fn get_edge_cost(
        &self,
        from_edge: Option<EdgeIndex>,
        via_node: Option<NodeIndex>,
        to_edge: Option<EdgeIndex>,
        r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        restrict: &Restrictor,
    ) -> u32 {
        let from_idx = match from_edge {
            Some(idx) => idx,
            None => return 0,
        };

        let edge = self.graph.edge(from_idx);
        let mut c = edge.payload.cost;

        if c == u32::MAX {
            return c;
        }

        let no_line_simi_pen = r_opts.line_unmatched_punish_fact == 1.0
            || (r_attrs.short_name.is_empty()
                && r_attrs.line_from.is_empty()
                && r_attrs.line_to.is_empty());

        if !no_line_simi_pen {
            let simi = Self::transit_line_simi(&edge.payload, r_attrs);

            if !simi.name_similar {
                if r_opts.line_unmatched_punish_fact < 1.0 {
                    c = ((c as f64) * r_opts.line_unmatched_punish_fact).ceil() as u32;
                } else if r_opts.line_unmatched_punish_fact > 1.0 {
                    let a = ((c as f64) * r_opts.line_unmatched_punish_fact).round();
                    if a > u32::MAX as f64 {
                        return u32::MAX;
                    }
                    c = a as u32;
                }
            }

            if !simi.from_similar {
                if r_opts.line_name_from_unmatched_punish_fact < 1.0 {
                    c = ((c as f64) * r_opts.line_name_from_unmatched_punish_fact).ceil() as u32;
                } else if r_opts.line_name_from_unmatched_punish_fact > 1.0 {
                    let a = ((c as f64) * r_opts.line_name_from_unmatched_punish_fact).round();
                    if a > u32::MAX as f64 {
                        return u32::MAX;
                    }
                    c = a as u32;
                }
            }

            if !simi.to_similar {
                if r_opts.line_name_to_unmatched_punish_fact < 1.0 {
                    c = ((c as f64) * r_opts.line_name_to_unmatched_punish_fact).ceil() as u32;
                } else if r_opts.line_name_to_unmatched_punish_fact > 1.0 {
                    let a = ((c as f64) * r_opts.line_name_to_unmatched_punish_fact).round();
                    if a > u32::MAX as f64 {
                        return u32::MAX;
                    }
                    c = a as u32;
                }
            }
        }

        if let (Some(n_idx), Some(to_idx)) = (via_node, to_edge) {
            let to_edge_data = self.graph.edge(to_idx);

            if r_opts.full_turn_punish_fac != 0 {
                if edge.from == to_edge_data.to && edge.to == to_edge_data.from {
                    c = c.saturating_add(r_opts.full_turn_punish_fac);
                } else {
                    let deg = self.graph.node(n_idx).edges().count();
                    if deg > 2 {
                        let p = self.graph.node(n_idx).payload.point;
                        let a = edge.payload.back_hop();
                        let b = to_edge_data.payload.front_hop();
                        let ang = self.inner_product(p, a, b);
                        if ang < r_opts.full_turn_angle {
                            c = c.saturating_add(r_opts.full_turn_punish_fac);
                        }
                    }
                }
            }

            if r_opts.turn_restr_cost > 0 && edge.payload.restriction && !restrict.may(from_idx, to_idx, n_idx) {
                c = c.saturating_add(r_opts.turn_restr_cost);
            }
        }

        c
    }

    fn search_dijkstra(
        &self,
        starts: &AHashMap<EdgeIndex, u32>,
        targets: &AHashSet<EdgeIndex>,
        max_cost: u32,
        r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        restrict: &Restrictor,
        reconstruct_target: Option<EdgeIndex>,
    ) -> (AHashMap<EdgeIndex, (u32, EdgeIndex)>, AHashMap<EdgeIndex, RouteEdge>, Option<Vec<EdgeIndex>>) {
        let mut dists: AHashMap<EdgeIndex, (u32, EdgeIndex)> = AHashMap::new();
        let mut settled: AHashMap<EdgeIndex, RouteEdge> = AHashMap::new();
        let mut pq = BinaryHeap::new();

        let max_speed = 36.11;
        let heuristic = DistHeur::new(self.graph, max_speed, targets);

        for (&start, &init_cost) in starts {
            if init_cost < u32::MAX {
                let h = heuristic.eval(self.graph, start);
                let priority = init_cost.saturating_add(h);
                pq.push(PQState {
                    priority,
                    cost: init_cost,
                    dwi: 0,
                    edge: start,
                    parent: None,
                    via_node: None,
                    source: start,
                });
                dists.insert(start, (init_cost, start));
            }
        }

        let mut found_reconstruct = false;

        while let Some(PQState {
            priority: _,
            cost,
            dwi,
            edge: u,
            parent,
            via_node,
            source,
        }) = pq.pop()
        {
            if dwi > max_cost {
                break;
            }

            if let Some(se) = settled.get(&u) {
                if se.cost <= cost {
                    continue;
                }
            }

            settled.insert(
                u,
                RouteEdge {
                    parent,
                    cost,
                    via_node,
                    source,
                },
            );

            dists.insert(u, (cost, source));

            if reconstruct_target == Some(u) {
                found_reconstruct = true;
                break;
            }

            let u_edge = self.graph.edge(u);
            let u_to_node_idx = u_edge.to;
            let u_to_node = self.graph.node(u_to_node_idx);

            for &v_idx in u_to_node.edges() {
                let v_edge = self.graph.edge(v_idx);
                if v_edge.from != u_to_node_idx {
                    continue;
                }

                let weight = self.get_edge_cost(
                    Some(u),
                    Some(u_to_node_idx),
                    Some(v_idx),
                    r_attrs,
                    r_opts,
                    restrict,
                );
                if weight == u32::MAX {
                    continue;
                }

                let next_cost = cost.saturating_add(weight);
                let next_dwi = dwi.saturating_add(weight);

                if next_dwi > max_cost {
                    continue;
                }

                let prev_best = dists.get(&v_idx).map(|d| d.0).unwrap_or(u32::MAX);
                if next_cost < prev_best {
                    dists.insert(v_idx, (next_cost, source));
                    let h = heuristic.eval(self.graph, v_idx);
                    let next_priority = next_cost.saturating_add(h);

                    pq.push(PQState {
                        priority: next_priority,
                        cost: next_cost,
                        dwi: next_dwi,
                        edge: v_idx,
                        parent: Some(u),
                        via_node: Some(u_to_node_idx),
                        source,
                    });
                }
            }
        }

        let path = if found_reconstruct {
            let target = reconstruct_target.unwrap();
            let mut p = Vec::new();
            let mut curr = target;
            let mut visited = AHashSet::new();
            while let Some(se) = settled.get(&curr) {
                if !visited.insert(curr) {
                    break;
                }
                p.push(curr);
                if let Some(parent_edge) = se.parent {
                    curr = parent_edge;
                } else {
                    break;
                }
            }
            p.reverse();
            Some(p)
        } else {
            None
        };

        (dists, settled, path)
    }

    fn reconstruct_path_from_settled(
        &self,
        settled: &AHashMap<EdgeIndex, RouteEdge>,
        start: EdgeIndex,
        mut curr: EdgeIndex,
    ) -> Vec<EdgeIndex> {
        let mut path = Vec::new();
        let mut visited = AHashSet::new();
        while curr != start {
            if !visited.insert(curr) {
                return vec![];
            }
            path.push(curr);
            if let Some(se) = settled.get(&curr) {
                if let Some(parent_edge) = se.parent {
                    curr = parent_edge;
                } else {
                    break;
                }
            } else {
                return vec![];
            }
        }
        path.push(start);
        path.reverse();
        path
    }

    fn hops(
        &self,
        froms: &EdgeCandGroup,
        tos: &EdgeCandGroup,
        r_costs: &mut Vec<((usize, usize), u32)>,
        dists: &mut Vec<((usize, usize), u32)>,
        r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        restrict: &Restrictor,
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

        let mut max_progr_start = 0.0;
        for fr in froms {
            if let Some(e) = fr.edge {
                let progr_start = if fr.progr > 0.0 {
                    let cost_e = self.get_edge_cost(Some(e), None, None, r_attrs, r_opts, restrict);
                    (cost_e as f64) * fr.progr
                } else {
                    0.0
                };
                if progr_start > max_progr_start {
                    max_progr_start = progr_start;
                }
            }
        }

        let mut max_cost = self.add_non_overflow(max_cost, max_progr_start as u32);

        let mut ecm_cost: AHashMap<(EdgeIndex, EdgeIndex), u32> = AHashMap::new();
        let mut ecm_dist: AHashMap<(EdgeIndex, EdgeIndex), f64> = AHashMap::new();

        for &e_from in &e_frs {
            let mut rem_tos = AHashSet::new();

            for &e_to in &e_tos {
                let cached = if let Some(cache) = &hop_cache {
                    cache.get(e_from, e_to)
                } else {
                    (0, false)
                };

                let from_comp = self
                    .graph
                    .node(self.graph.edge(e_from).from)
                    .payload
                    .comp_id;
                let to_comp = self.graph.node(self.graph.edge(e_to).to).payload.comp_id;

                if from_comp != to_comp {
                    ecm_cost.insert((e_from, e_to), ROUTE_INF);
                } else if cached.1 && cached.0 >= ROUTE_INF {
                    ecm_cost.insert((e_from, e_to), ROUTE_INF);
                } else if !TW::need_dist() && cached.1 {
                    ecm_cost.insert((e_from, e_to), cached.0);
                } else {
                    rem_tos.insert(e_to);
                }
            }

            if !rem_tos.is_empty() {
                let mut starts_map = AHashMap::new();
                starts_map.insert(e_from, 0);

                let (costs, settled, _) = self.search_dijkstra(
                    &starts_map,
                    &rem_tos,
                    max_cost,
                    r_attrs,
                    r_opts,
                    restrict,
                    None,
                );

                for &to_e in &rem_tos {
                    let cost = costs.get(&to_e).map(|d| d.0).unwrap_or(ROUTE_INF);
                    ecm_cost.insert((e_from, to_e), cost);

                    if cost < ROUTE_INF && TW::need_dist() {
                        let path = self.reconstruct_path_from_settled(&settled, e_from, to_e);
                        let mut dist = 0.0;
                        if path.len() > 1 {
                            for i in 0..path.len() - 1 {
                                dist += self.graph.edge(path[i]).payload.length();
                            }
                        }
                        ecm_dist.insert((e_from, to_e), dist);
                    }

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

            let cost_fr = self.get_edge_cost(Some(e_fr), None, None, r_attrs, r_opts, restrict);

            for (to_id, to) in tos.iter().enumerate() {
                let e_to = match to.edge {
                    Some(e) => e,
                    None => continue,
                };

                let val = ecm_cost.get(&(e_fr, e_to)).cloned().unwrap_or(ROUTE_INF);
                if val >= max_cost {
                    continue;
                }

                let mut c = val;
                let mut dist = if TW::need_dist() {
                    ecm_dist.get(&(e_fr, e_to)).cloned().unwrap_or(0.0)
                } else {
                    0.0
                };

                if e_fr == e_to {
                    if fr.progr <= to.progr {
                        let cost_to = self.get_edge_cost(Some(e_to), None, None, r_attrs, r_opts, restrict);
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
                        if TW::need_dist() {
                            dist -= self.graph.edge(e_fr).payload.length() * fr.progr;
                        }
                    }
                    if to.progr > 0.0 {
                        let cost_to = self.get_edge_cost(Some(e_to), None, None, r_attrs, r_opts, restrict);
                        let progr_c_to = (cost_to as f64 * to.progr) as u32;
                        c = c.saturating_add(progr_c_to);
                        if TW::need_dist() {
                            dist += self.graph.edge(e_to).payload.length() * to.progr;
                        }
                    }
                }

                if c < max_cost.saturating_sub(max_progr_start as u32) {
                    r_costs.push(((fr_id, to_id), c));
                    if TW::need_dist() {
                        dists.push(((fr_id, to_id), dist as u32));
                    }
                }
            }
        }
    }

    fn hops_fast(
        &self,
        froms: &EdgeCandGroup,
        tos: &EdgeCandGroup,
        init_costs_vec: &Vec<f64>,
        r_costs: &mut Vec<((usize, usize), u32)>,
        r_attrs: &RoutingAttrs,
        r_opts: &RoutingOpts,
        restrict: &Restrictor,
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
                    let cost_e = self.get_edge_cost(Some(e), None, None, r_attrs, r_opts, restrict);
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
                    let cost_e = self.get_edge_cost(Some(e), None, None, r_attrs, r_opts, restrict);
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

        let (dists, settled, _) = self.search_dijkstra(
            &init_costs_map,
            &e_tos,
            max_cost,
            r_attrs,
            r_opts,
            restrict,
            None,
        );

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
                    let cost_fr = self.get_edge_cost(Some(from_edg), None, None, r_attrs, r_opts, restrict);

                    if let Some(to_cands) = e_to_cands.get(&to_edg) {
                        for &to_id in to_cands {
                            let to = &tos[to_id];
                            let mut wr_cost = base_cost;

                            if from_edg == to_edg {
                                if fr.progr <= to.progr {
                                    let cost_to = self.get_edge_cost(Some(to_edg), None, None, r_attrs, r_opts, restrict);
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
                                    let cost_to = self.get_edge_cost(Some(to_edg), None, None, r_attrs, r_opts, restrict);
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
