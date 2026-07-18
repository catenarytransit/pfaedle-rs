use crate::router::weights::RoutingAttrs;
use ahash::AHashMap;
use geo::Point;
use gtfs_structures::Trip;

#[derive(Debug, Clone)]
pub struct TripTrieNd {
    pub stop_name: String,
    pub platform: String,
    pub pos: Point<f64>,
    pub lat: f64,
    pub lng: f64,
    pub time: i32,
    pub arr: bool,
    pub trip_time: i32,
    pub trips: u32,
    pub parent: Option<usize>,
    pub childs: Vec<usize>,
    pub r_attrs: RoutingAttrs,
    pub acc_time: i64,
}

pub struct TripTrie {
    nds: Vec<TripTrieNd>,
    nd_trips: AHashMap<usize, Vec<String>>,
    trip_nds: AHashMap<String, usize>,
}

impl TripTrie {
    pub fn new() -> Self {
        let root = TripTrieNd {
            stop_name: "ROOT".to_string(),
            platform: "".to_string(),
            pos: Point::new(0.0, 0.0),
            lat: 0.0,
            lng: 0.0,
            time: 0,
            arr: false,
            trip_time: 0,
            trips: 0,
            parent: None,
            childs: Vec::new(),
            r_attrs: RoutingAttrs::default(),
            acc_time: 0,
        };

        Self {
            nds: vec![root],
            nd_trips: AHashMap::new(),
            trip_nds: AHashMap::new(),
        }
    }

    pub fn get_nds(&self) -> &Vec<TripTrieNd> {
        &self.nds
    }

    pub fn get_nd(&self, index: usize) -> &TripTrieNd {
        &self.nds[index]
    }

    pub fn get_nd_trips(&self) -> &AHashMap<usize, Vec<String>> {
        &self.nd_trips
    }

    pub fn add_trip(
        &mut self,
        trip: &Trip,
        r_attrs: &RoutingAttrs,
        time_ex: bool,
        degen: bool,
    ) -> bool {
        if !degen {
            return self.add(trip, r_attrs, time_ex);
        }

        if let Some(existing) = self.get(trip, time_ex) {
            if self.nds[existing].childs.is_empty() {
                self.trip_nds.insert(trip.id.clone(), existing);
                self.nd_trips
                    .entry(existing)
                    .or_default()
                    .push(trip.id.clone());
                return true;
            }
            return false;
        } else {
            return false;
        }
    }

    fn add(&mut self, trip: &Trip, r_attrs: &RoutingAttrs, time_ex: bool) -> bool {
        if trip.stop_times.is_empty() {
            return false;
        }

        let start_secs = trip.stop_times[0].departure_time.unwrap_or(0);
        let mut cur_nd_id = 0;

        for (st_id, st) in trip.stop_times.iter().enumerate() {
            let stop = &st.stop;
            // stop.name is Option<String>
            let name_string = stop.name.clone().unwrap_or_default();
            let name = name_string.as_str();

            let platform_val = stop.code.clone().unwrap_or_default();
            let platform: &str = &platform_val;

            let lat = stop.latitude.unwrap_or(0.0);
            let lng = stop.longitude.unwrap_or(0.0);
            let pos = Point::new(lng, lat);

            if st_id > 0 {
                let arr_time = if let Some(t) = st.arrival_time {
                    (t - start_secs) as i32
                } else {
                    (start_secs - start_secs) as i32
                };

                if let Some(arr_child) =
                    self.get_match_child(cur_nd_id, name, platform, pos, arr_time, time_ex)
                {
                    cur_nd_id = arr_child;
                    self.nds[arr_child].acc_time += arr_time as i64;
                    self.nds[arr_child].trips += 1;
                    self.nds[arr_child].r_attrs.merge(r_attrs);
                } else {
                    cur_nd_id = self.insert(stop, r_attrs, pos, arr_time, true, cur_nd_id);
                }
            }

            if st_id < trip.stop_times.len() - 1 {
                let dep_time = if let Some(t) = st.departure_time {
                    (t - start_secs) as i32
                } else {
                    0
                };

                if let Some(dep_child) =
                    self.get_match_child(cur_nd_id, name, platform, pos, dep_time, time_ex)
                {
                    cur_nd_id = dep_child;
                    self.nds[dep_child].acc_time += dep_time as i64;
                    self.nds[dep_child].trips += 1;
                    self.nds[dep_child].r_attrs.merge(r_attrs);
                } else {
                    if st_id == 0 && !self.trip_nds.is_empty() {
                        return false;
                    }
                    cur_nd_id = self.insert(stop, r_attrs, pos, dep_time, false, cur_nd_id);
                }
            }
        }

        self.trip_nds.insert(trip.id.clone(), cur_nd_id);
        self.nd_trips
            .entry(cur_nd_id)
            .or_default()
            .push(trip.id.clone());
        true
    }

    pub fn get(&self, trip: &Trip, time_ex: bool) -> Option<usize> {
        if trip.stop_times.is_empty() {
            return None;
        }
        let start_secs = trip.stop_times[0].departure_time.unwrap_or(0);
        let mut cur_nd_id = 0;

        for (st_id, st) in trip.stop_times.iter().enumerate() {
            let stop = &st.stop;
            let name_string = stop.name.clone().unwrap_or_default();
            let name = name_string.as_str();

            let platform_val = stop.code.clone().unwrap_or_default();
            let platform: &str = &platform_val;

            let lat = stop.latitude.unwrap_or(0.0);
            let lng = stop.longitude.unwrap_or(0.0);
            let pos = Point::new(lng, lat);

            if st_id > 0 {
                let arr_time = if let Some(t) = st.arrival_time {
                    (t - start_secs) as i32
                } else {
                    0
                };

                match self.get_match_child(cur_nd_id, name, platform, pos, arr_time, time_ex) {
                    Some(child) => cur_nd_id = child,
                    None => return None,
                }
            }

            if st_id < trip.stop_times.len() - 1 {
                let dep_time = if let Some(t) = st.departure_time {
                    (t - start_secs) as i32
                } else {
                    0
                };

                match self.get_match_child(cur_nd_id, name, platform, pos, dep_time, time_ex) {
                    Some(child) => cur_nd_id = child,
                    None => return None,
                }
            }
        }
        Some(cur_nd_id)
    }

    fn get_match_child(
        &self,
        parent_nid: usize,
        stop_name: &str,
        platform: &str,
        pos: Point<f64>,
        time: i32,
        time_ex: bool,
    ) -> Option<usize> {
        use geo::HaversineDistance;

        for &child in &self.nds[parent_nid].childs {
            let nd = &self.nds[child];
            let dist = nd.pos.haversine_distance(&pos);

            if nd.stop_name == stop_name
                && nd.platform == platform
                && dist < 1.0
                && (!time_ex || nd.time == time)
            {
                return Some(child);
            }
        }
        None
    }

    fn insert(
        &mut self,
        stop: &gtfs_structures::Stop,
        r_attrs: &RoutingAttrs,
        pos: Point<f64>,
        time: i32,
        arr: bool,
        parent: usize,
    ) -> usize {
        let node = TripTrieNd {
            stop_name: stop.name.clone().unwrap_or_default(),
            platform: stop.code.as_ref().cloned().unwrap_or_default(),
            pos,
            lat: stop.latitude.unwrap_or(0.0),
            lng: stop.longitude.unwrap_or(0.0),
            time,
            arr,
            trip_time: time,
            trips: 1,
            parent: Some(parent),
            childs: Vec::new(),
            r_attrs: r_attrs.clone(),
            acc_time: time as i64,
        };

        let idx = self.nds.len();
        self.nds.push(node);
        self.nds[parent].childs.push(idx);
        idx
    }

    #[cfg(test)]
    pub fn new_dummy(nds: Vec<TripTrieNd>) -> Self {
        Self {
            nds,
            nd_trips: AHashMap::new(),
            trip_nds: AHashMap::new(),
        }
    }
}
