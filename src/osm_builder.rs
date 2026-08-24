use ahash::{AHashMap, AHashSet};
use anyhow::{Context, Result};
use geo::{LineString, Point};
use gtfs_structures::RouteType;
use osmpbfreader::{OsmId, OsmObj, OsmPbfReader, Tags};
use rayon::prelude::*;
use rstar::RTree;
use std::path::Path;

use crate::graph::{
    EdgeIndex, EdgePL, Graph, MODE_BUS, MODE_FERRY, MODE_GONDOLA, MODE_RAIL, MODE_SUBWAY,
    MODE_TRAM, NodeIndex, NodePL, TransitInfo, TransitInfoInterner,
};
use crate::osm_filter::OsmFilter;
use crate::upstream_graph::{
    NODE_FLAG_RESTRICTION, NODE_FLAG_STATION, apply_one_way_penalty, collapse_edges, node_flags,
    write_components, write_other_direction_edges,
};

#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SpatialNode {
    pub index: NodeIndex,
    pub point: [f64; 2],
    pub modes: u8,
}

impl rstar::PointDistance for SpatialNode {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        let dx = self.point[0] - point[0];
        let dy = self.point[1] - point[1];
        dx * dx + dy * dy
    }
}

impl rstar::RTreeObject for SpatialNode {
    type Envelope = rstar::AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        rstar::AABB::from_point(self.point)
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct OsmRelation {
    pub id: i64,
    pub tags: Tags,
    pub nodes: Vec<NodeIndex>,
    pub edges: AHashSet<EdgeIndex>,
}

#[derive(Clone)]
pub struct OsmData {
    pub graph: Graph<NodePL, EdgePL>,
    pub timestamp: String,
    pub spatial_tree: Option<RTree<SpatialNode>>,
    pub osm_filepath: std::path::PathBuf,
    pub relations: Vec<OsmRelation>,
    pub node_to_relations: AHashMap<NodeIndex, Vec<usize>>,
}

#[derive(Debug, Clone)]
pub struct LightRelation {
    pub ref_tag: Option<String>,
    pub name: Option<String>,
    pub from_str: String,
    pub to_str: String,
    pub operator: Option<String>,
    pub colour: Option<String>,
}

#[derive(Debug, Clone)]
pub struct LightOsmData {
    pub relations: Vec<LightRelation>,
    pub way_to_relations: AHashMap<i64, Vec<usize>>,
}

impl LightOsmData {
    pub fn new() -> Self {
        Self {
            relations: Vec::new(),
            way_to_relations: AHashMap::new(),
        }
    }

    pub fn relations_for_way(&self, way_id: i64) -> impl Iterator<Item = &LightRelation> {
        self.way_to_relations
            .get(&way_id)
            .into_iter()
            .flat_map(|indices| indices.iter().map(|&index| &self.relations[index]))
    }

    pub fn find_color(
        &self,
        route_short_name: Option<&str>,
        route_long_name: Option<&str>,
        operator: Option<&str>,
    ) -> Option<String> {
        let mut best_match: Option<(&LightRelation, u8)> = None;
        for relation in &self.relations {
            if relation.colour.is_none() {
                continue;
            }
            let mut score = 0u8;
            if let Some(osm_ref) = &relation.ref_tag {
                let osm_lower = osm_ref.to_lowercase();
                if let Some(short) = route_short_name {
                    if osm_lower.contains(short) || short.contains(&osm_lower) {
                        score += 3;
                    }
                }
            }
            if let Some(osm_name) = &relation.name {
                let osm_lower = osm_name.to_lowercase();
                if let Some(short) = route_short_name {
                    if osm_lower.contains(short) || short.contains(&osm_lower) {
                        score += 2;
                    }
                }
                if let Some(long) = route_long_name {
                    if osm_lower.contains(long) || long.contains(&osm_lower) {
                        score += 2;
                    }
                }
            }
            if let (Some(osm_op), Some(gtfs_op)) = (&relation.operator, operator) {
                let osm_lower = osm_op.to_lowercase();
                if osm_lower.contains(gtfs_op) || gtfs_op.contains(&osm_lower) {
                    score += 1;
                }
            }
            if score > 0 && best_match.as_ref().map_or(true, |(_, old)| score > *old) {
                best_match = Some((relation, score));
            }
        }
        best_match.and_then(|(relation, _)| relation.colour.clone())
    }
}

#[derive(Debug, Clone)]
pub struct PreRelation {
    pub id: i64,
    pub tags: Tags,
    pub members: Vec<osmpbfreader::Ref>,
}

/// Data discovered before node/edge construction. C++ pfaedle performs the same
/// separation in its relation/way/node passes; keeping it explicit also lets the
/// streaming splitter reuse exactly the same filtered resource set.
pub struct IdentifiedResources {
    pub pre_relations: Vec<PreRelation>,
    pub ways_in_relations: AHashSet<i64>,
    pub ways_in_ferry_relations: AHashSet<i64>,
    pub needed_nodes: Vec<i64>,
    /// Sorted IDs of ways kept by the mode-specific resource pass. Keeping this
    /// flat (rather than as a hash set) makes the streaming splitter exact
    /// without retaining another large hash table.
    pub kept_ways: Vec<i64>,
    pub restricted_ways: AHashSet<i64>,
    pub restriction_via_nodes: AHashSet<i64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OsmProfile {
    Rail,
    TramSubway,
    Bus,
    Ferry,
    Gondola,
    Funicular,
    Generic(u8),
}

impl OsmProfile {
    fn allowed_modes(self) -> u8 {
        match self {
            OsmProfile::Rail => MODE_RAIL,
            OsmProfile::TramSubway => MODE_TRAM | MODE_SUBWAY,
            OsmProfile::Bus => MODE_BUS,
            OsmProfile::Ferry => MODE_FERRY,
            OsmProfile::Gondola | OsmProfile::Funicular => MODE_GONDOLA,
            OsmProfile::Generic(mask) => mask,
        }
    }

    fn one_way_penalty_factor(self) -> u32 {
        match self {
            OsmProfile::Rail | OsmProfile::Bus => 5,
            OsmProfile::TramSubway
            | OsmProfile::Ferry
            | OsmProfile::Gondola
            | OsmProfile::Funicular => 2,
            OsmProfile::Generic(_) => 5,
        }
    }

    fn one_way_entry_cost_seconds(self) -> u32 {
        // Default pfaedle.cfg explicitly sets 300s for rail and [bus, coach].
        // Other MOT configs leave the MotConfigReader default of zero.
        match self {
            OsmProfile::Rail | OsmProfile::Bus => 300,
            _ => 0,
        }
    }
}

pub struct OsmBuilder;

impl OsmBuilder {
    fn open_pbf(path: &Path) -> Result<OsmPbfReader<std::fs::File>> {
        let file =
            std::fs::File::open(path).with_context(|| format!("Failed to open {:?}", path))?;
        Ok(OsmPbfReader::new(file))
    }

    fn profile_for_route_types(used_route_types: &AHashSet<RouteType>) -> OsmProfile {
        use crate::mots::{MotCategory, map_route_type_to_category};

        let categories: Vec<_> = used_route_types
            .iter()
            .copied()
            .map(map_route_type_to_category)
            .collect();
        if !categories.is_empty()
            && categories
                .iter()
                .all(|category| *category == MotCategory::Rail)
        {
            return OsmProfile::Rail;
        }
        if !categories.is_empty()
            && categories
                .iter()
                .all(|category| matches!(*category, MotCategory::Tram | MotCategory::Subway))
        {
            return OsmProfile::TramSubway;
        }
        if !categories.is_empty()
            && categories
                .iter()
                .all(|category| *category == MotCategory::Ferry)
        {
            return OsmProfile::Ferry;
        }
        if !categories.is_empty()
            && categories
                .iter()
                .all(|category| matches!(*category, MotCategory::Gondola | MotCategory::CableCar))
        {
            return OsmProfile::Gondola;
        }
        if !categories.is_empty()
            && categories
                .iter()
                .all(|category| *category == MotCategory::Funicular)
        {
            return OsmProfile::Funicular;
        }

        // Compatibility fallback for route types that do not map to one of the
        // upstream configuration groups above.
        let mut mask = 0u8;
        for category in categories {
            mask |= match category {
                MotCategory::Rail => MODE_RAIL,
                MotCategory::Tram => MODE_TRAM,
                MotCategory::Subway => MODE_SUBWAY,
                MotCategory::Ferry => MODE_FERRY,
                MotCategory::Gondola | MotCategory::Funicular | MotCategory::CableCar => {
                    MODE_GONDOLA
                }
                _ => MODE_BUS,
            };
        }
        OsmProfile::Generic(mask)
    }

    fn relation_kept_for_profile(tags: &Tags, profile: OsmProfile) -> bool {
        let railway = tags.get("railway").map(|value| value.as_str());
        let highway = tags.get("highway").map(|value| value.as_str());
        let route = tags.get("route").map(|value| value.as_str());

        let dropped = match profile {
            OsmProfile::Rail => {
                matches!(
                    railway,
                    Some(
                        "abandoned"
                            | "construction"
                            | "disused"
                            | "miniature"
                            | "signal"
                            | "razed"
                            | "proposed"
                            | "platform"
                    )
                ) || tags.get("metro").map(|value| value.as_str()) == Some("yes")
                    || tags.get("area").map(|value| value.as_str()) == Some("yes")
                    || tags.get("type").map(|value| value.as_str()) == Some("multipolygon")
                    || tags.get("public_transport").map(|value| value.as_str()) == Some("platform")
                    || tags.get("building").is_some_and(|value| {
                        matches!(value.as_str(), "yes" | "train_station" | "roof")
                    })
                    || tags
                        .get("amenity")
                        .is_some_and(|value| matches!(value.as_str(), "shelter" | "bus_station"))
            }
            OsmProfile::TramSubway | OsmProfile::Gondola | OsmProfile::Funicular => {
                tags.get("area").map(|value| value.as_str()) == Some("yes")
                    || tags.get("public_transport").map(|value| value.as_str()) == Some("stop_area")
                    || tags.get("type").map(|value| value.as_str()) == Some("multipolygon")
                    || railway == Some("platform")
                    || tags.get("public_transport").map(|value| value.as_str()) == Some("platform")
                    || tags.get("service").map(|value| value.as_str()) == Some("alley")
            }
            OsmProfile::Bus => {
                // `train=yes|no_match_ways` still applies to relations, while
                // `public_transport=stop_area|no_match_nds|no_match_rels` does not.
                tags.get("area").map(|value| value.as_str()) == Some("yes")
                    || tags.get("train").map(|value| value.as_str()) == Some("yes")
                    || tags.get("type").map(|value| value.as_str()) == Some("multipolygon")
                    || matches!(railway, Some("platform" | "station"))
                    || matches!(highway, Some("proposed" | "footway" | "construction"))
                    || tags
                        .get("building")
                        .is_some_and(|value| matches!(value.as_str(), "yes" | "train_station"))
                    || tags
                        .get("leisure")
                        .is_some_and(|value| matches!(value.as_str(), "garden" | "park"))
            }
            OsmProfile::Ferry => false,
            OsmProfile::Generic(_) => false,
        };
        if dropped {
            return false;
        }

        match profile {
            OsmProfile::Rail => {
                matches!(
                    railway,
                    Some("rail" | "light_rail" | "tram" | "narrow_gauge")
                ) || matches!(route, Some("rail" | "light_rail" | "train"))
                    || tags.get("public_transport").map(|value| value.as_str()) == Some("stop_area")
            }
            OsmProfile::TramSubway => {
                matches!(route, Some("tram" | "funicular" | "subway" | "light_rail"))
                    || matches!(
                        railway,
                        Some(
                            "subway"
                                | "light_rail"
                                | "tram"
                                | "funicular"
                                | "station"
                                | "halt"
                                | "tram_stop"
                        )
                    )
                    || tags.get("subway").map(|value| value.as_str()) == Some("yes")
                    || tags.get("tram").map(|value| value.as_str()) == Some("yes")
            }
            OsmProfile::Bus => {
                let bus_keep = matches!(
                    highway,
                    Some(
                        "motorway"
                            | "trunk"
                            | "primary"
                            | "secondary"
                            | "tertiary"
                            | "residential"
                            | "living_street"
                            | "unclassified"
                            | "motorway_link"
                            | "trunk_link"
                            | "primary_link"
                            | "secondary_link"
                            | "tertiary_link"
                            | "residential_link"
                            | "bus_guideway"
                    )
                ) || tags.get("way").is_some_and(|value| {
                    matches!(value.as_str(), "primary" | "seconday" | "bus_guideway")
                }) || tags.contains_key("busway")
                    || tags
                        .get("psv")
                        .is_some_and(|value| matches!(value.as_str(), "yes" | "designated"))
                    || tags
                        .get("bus")
                        .is_some_and(|value| matches!(value.as_str(), "yes" | "designated"))
                    || tags
                        .get("minibus")
                        .is_some_and(|value| matches!(value.as_str(), "yes" | "designated"))
                    || tags.get("trolley_wire").map(|value| value.as_str()) == Some("yes")
                    || tags.get("trolleywire").map(|value| value.as_str()) == Some("yes")
                    || tags.get("trolleybus").map(|value| value.as_str()) == Some("yes")
                    || tags.get("trolley_bus").map(|value| value.as_str()) == Some("yes")
                    || matches!(route, Some("bus" | "trolleybus"))
                    || tags
                        .get("bus:lanes")
                        .is_some_and(|value| matches!(value.as_str(), "yes" | "designated" | "1"))
                    || tags
                        .get("lanes:bus")
                        .is_some_and(|value| matches!(value.as_str(), "1" | "2" | "3"))
                    || tags
                        .get("lanes:psv")
                        .is_some_and(|value| matches!(value.as_str(), "1" | "2" | "3"))
                    || tags.get("public_transport").map(|value| value.as_str())
                        == Some("stop_position")
                    || tags.contains_key("bus_stop")
                    || tags.contains_key("stop")
                    || highway == Some("bus_stop");

                bus_keep
                    || matches!(
                        tags.get("type").map(|value| value.as_str()),
                        Some("restriction" | "restriction:bus" | "restriction:motorcar")
                    )
            }
            OsmProfile::Ferry => {
                route == Some("ferry")
                    || tags.get("waterway").map(|value| value.as_str()) == Some("river")
                    || tags.get("motorboat").map(|value| value.as_str()) == Some("yes")
                    || tags.get("ferry").map(|value| value.as_str()) == Some("yes")
                    || tags.get("amenity").map(|value| value.as_str()) == Some("ferry_terminal")
                    || tags.get("mooring").map(|value| value.as_str()) == Some("ferry")
                    || tags.get("station").map(|value| value.as_str()) == Some("ferry")
            }
            OsmProfile::Gondola => matches!(
                tags.get("aerialway").map(|value| value.as_str()),
                Some("gondola" | "cable_car" | "chair_lift" | "mixed_lift")
            ),
            OsmProfile::Funicular => {
                matches!(route, Some("funicular" | "tram" | "subway" | "light_rail"))
                    || matches!(
                        railway,
                        Some(
                            "funicular"
                                | "narrow_gauge"
                                | "subway"
                                | "light_rail"
                                | "tram"
                                | "station"
                                | "halt"
                                | "tram_stop"
                        )
                    )
                    || tags.get("subway").map(|value| value.as_str()) == Some("yes")
                    || tags.get("tram").map(|value| value.as_str()) == Some("yes")
            }
            OsmProfile::Generic(_) => tags.contains_key("route"),
        }
    }

    fn relation_is_flat(tags: &Tags, profile: OsmProfile) -> bool {
        // The only default MOT rule with `rel_flat` is rail
        // `public_transport=stop_area|rel_flat`.
        profile == OsmProfile::Rail
            && tags.get("public_transport").map(|value| value.as_str()) == Some("stop_area")
    }

    fn bus_restriction_relation_kept(tags: &Tags) -> bool {
        // Upstream [bus, coach] keeps only these relation types for its
        // restriction system and then applies the positive/negative restriction
        // filters. `except=bus` / `except=psv` disables the restriction.
        // C++ readRestr() has an additional exact type check after the MOT
        // relation filter: only type=restriction enters the Restrictor. The
        // restriction:bus/motorcar values are retained as ordinary kept
        // relations but do not mark graph edges restricted upstream.
        if tags.get("type").map(|value| value.as_str()) != Some("restriction") {
            return false;
        }

        if tags.get("except").is_some_and(|value| {
            value
                .split(';')
                .map(str::trim)
                .any(|part| matches!(part, "bus" | "psv"))
        }) {
            return false;
        }

        const NEGATIVE: &[&str] = &[
            "no_right_turn",
            "no_left_turn",
            "no_u_turn",
            "no_straight_on",
        ];
        const POSITIVE: &[&str] = &["only_left_turn", "only_straight_on", "only_right_turn"];
        let accepted = |value: &str| NEGATIVE.contains(&value) || POSITIVE.contains(&value);
        tags.get("restriction")
            .is_some_and(|value| accepted(value.as_str()))
            || tags
                .get("restriction:bus")
                .is_some_and(|value| accepted(value.as_str()))
    }

    pub fn identify_resources(path: &Path, skip_small_roads: bool) -> Result<IdentifiedResources> {
        Self::identify_resources_for_profile(path, skip_small_roads, OsmProfile::Bus)
    }

    pub fn identify_resources_for_modes(
        path: &Path,
        skip_small_roads: bool,
        mode_mask: u8,
    ) -> Result<IdentifiedResources> {
        let profile = if mode_mask == MODE_BUS {
            OsmProfile::Bus
        } else {
            OsmProfile::Generic(mode_mask)
        };
        Self::identify_resources_for_profile(path, skip_small_roads, profile)
    }

    fn identify_resources_for_profile(
        path: &Path,
        skip_small_roads: bool,
        profile: OsmProfile,
    ) -> Result<IdentifiedResources> {
        let mut pre_relations = Vec::new();
        let mut ways_in_relations = AHashSet::new();
        let mut ways_in_ferry_relations = AHashSet::new();
        let mut needed_nodes = Vec::new();
        let mut kept_ways = Vec::new();
        let mut restricted_ways = AHashSet::new();
        let mut restriction_via_nodes = AHashSet::new();

        println!("Pass 1/4: Scanning relations...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 1")?;
                let OsmObj::Relation(relation) = obj else {
                    continue;
                };

                if matches!(profile, OsmProfile::Bus | OsmProfile::Generic(_))
                    && Self::bus_restriction_relation_kept(&relation.tags)
                {
                    for member in &relation.refs {
                        match member.member {
                            OsmId::Way(way_id) if member.role == "from" || member.role == "to" => {
                                restricted_ways.insert(way_id.0);
                            }
                            OsmId::Node(node_id) if member.role == "via" => {
                                restriction_via_nodes.insert(node_id.0);
                                needed_nodes.push(node_id.0);
                            }
                            _ => {}
                        }
                    }
                }

                // C++ readRels stores every relation admitted by the MOT
                // filter. Its `rel_flat` flag only suppresses downward keeping;
                // way->relation metadata is still available to getLines().
                if !Self::relation_kept_for_profile(&relation.tags, profile) {
                    continue;
                }
                let flat = Self::relation_is_flat(&relation.tags, profile);

                for member in &relation.refs {
                    match member.member {
                        OsmId::Way(way_id) => {
                            if !flat {
                                ways_in_relations.insert(way_id.0);
                                if relation.tags.get("route").is_some_and(|v| v == "ferry") {
                                    ways_in_ferry_relations.insert(way_id.0);
                                }
                            }
                        }
                        OsmId::Node(node_id) if !flat => needed_nodes.push(node_id.0),
                        _ => {}
                    }
                }

                pre_relations.push(PreRelation {
                    id: relation.id.0,
                    tags: relation.tags,
                    members: relation.refs,
                });
            }
        }

        println!(
            "  Found {} relevant relations. Need {} relation ways.",
            pre_relations.len(),
            ways_in_relations.len()
        );

        println!("Pass 2/4: Scanning ways to identify needed nodes...");
        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 2")?;
                let OsmObj::Way(way) = obj else {
                    continue;
                };
                let way_id = way.id.0;
                let is_relation_member = ways_in_relations.contains(&way_id)
                    && !Self::way_dropped_for_profile(&way, profile);
                let is_infrastructure =
                    Self::way_kept_for_profile(&way, profile, &ways_in_ferry_relations);

                if skip_small_roads && is_infrastructure && !is_relation_member {
                    let highway = way.tags.get("highway").map(|v| v.as_str());
                    let service = way.tags.get("service").map(|v| v.as_str());
                    if highway == Some("residential")
                        || highway == Some("service")
                        || service == Some("driveway")
                    {
                        continue;
                    }
                }

                if (!is_infrastructure && !is_relation_member) || !Self::is_valid_way(&way) {
                    continue;
                }
                kept_ways.push(way_id);
                needed_nodes.extend(way.nodes.iter().map(|node_id| node_id.0));
            }
        }

        println!("  Sorting {} needed nodes...", needed_nodes.len());
        needed_nodes.par_sort_unstable();
        needed_nodes.dedup();
        kept_ways.par_sort_unstable();
        kept_ways.dedup();
        println!(
            "  Identified {} unique graph nodes across {} kept ways.",
            needed_nodes.len(),
            kept_ways.len()
        );

        Ok(IdentifiedResources {
            pre_relations,
            ways_in_relations,
            ways_in_ferry_relations,
            needed_nodes,
            kept_ways,
            restricted_ways,
            restriction_via_nodes,
        })
    }

    pub fn read(
        path: &Path,
        used_route_types: &AHashSet<RouteType>,
        bbox: Option<(f64, f64, f64, f64)>,
        skip_small_roads: bool,
    ) -> Result<OsmData> {
        println!("Reading OSM file {:?} in upstream-style passes...", path);
        let profile = Self::profile_for_route_types(used_route_types);
        let IdentifiedResources {
            pre_relations,
            ways_in_relations,
            ways_in_ferry_relations,
            mut needed_nodes,
            kept_ways,
            restricted_ways,
            restriction_via_nodes,
        } = Self::identify_resources_for_profile(path, skip_small_roads, profile)?;

        println!("Pass 3/4: Loading nodes...");
        let mut graph = Graph::new();
        let mut osm_node_to_graph_idx: AHashMap<i64, NodeIndex> = AHashMap::new();
        let mut graph_node_flags: Vec<u8> = Vec::new();

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 3")?;
                let OsmObj::Node(node) = obj else {
                    continue;
                };
                let node_id = node.id.0;
                if needed_nodes.binary_search(&node_id).is_err() {
                    continue;
                }

                let lat = node.lat();
                let lon = node.lon();
                if let Some((min_lon, min_lat, max_lon, max_lat)) = bbox {
                    if lon < min_lon || lon > max_lon || lat < min_lat || lat > max_lat {
                        continue;
                    }
                }

                let index = graph.add_node(NodePL {
                    comp_id: 0,
                    point: Point::new(lon, lat),
                });
                osm_node_to_graph_idx.insert(node_id, index);
                let mut flags = Self::node_flags_for_profile(&node.tags, profile);
                if restriction_via_nodes.contains(&node_id) {
                    flags |= NODE_FLAG_RESTRICTION;
                }
                graph_node_flags.push(flags);
            }
        }
        needed_nodes.clear();
        needed_nodes.shrink_to_fit();
        println!("  Raw graph has {} nodes.", graph.nodes.len());

        println!("Pass 4/4: Building forward edges and relation metadata...");
        // Direct analogue of C++ OsmBuilder::_lines/_relLines: canonicalize the
        // strings once and let every edge retain only compact shared identities.
        let mut line_interner = TransitInfoInterner::new();
        let mut way_transit_info: AHashMap<i64, Vec<u32>> = AHashMap::new();
        for relation in &pre_relations {
            if let Some(info) = Self::transit_info_for_relation(relation) {
                let info_id = line_interner.intern(info);
                for member in &relation.members {
                    if let OsmId::Way(way_id) = member.member {
                        way_transit_info.entry(way_id.0).or_default().push(info_id);
                    }
                }
            }
        }
        for ids in way_transit_info.values_mut() {
            ids.sort_unstable();
            ids.dedup();
        }
        graph.transit_lines = line_interner.into_infos();
        // All relation string/member data has now been compiled into the
        // compact way->line-ID table. Upstream likewise needs it only during
        // graph construction, so release it before the edge PBF pass.
        drop(pre_relations);

        {
            let mut pbf = Self::open_pbf(path)?;
            for obj in pbf.iter() {
                let obj = obj.context("Error reading PBF object in Pass 4")?;
                let OsmObj::Way(way) = obj else {
                    continue;
                };
                let way_id = way.id.0;
                if kept_ways.binary_search(&way_id).is_err() {
                    continue;
                }
                let is_relation_member = ways_in_relations.contains(&way_id)
                    && !Self::way_dropped_for_profile(&way, profile);
                let modes = profile.allowed_modes();
                let is_infrastructure =
                    Self::way_kept_for_profile(&way, profile, &ways_in_ferry_relations)
                        || is_relation_member;
                if (!is_infrastructure && !is_relation_member) || !Self::is_valid_way(&way) {
                    continue;
                }

                let transit_lines = way_transit_info.get(&way_id).cloned().unwrap_or_default();
                // Match upstream readEdges(): walk the graph nodes of the kept
                // way in OSM order. Nodes outside the active bounding graph are
                // skipped, while the previous kept node remains the predecessor.
                let way_nodes: Vec<(i64, NodeIndex)> = way
                    .nodes
                    .iter()
                    .filter_map(|node_id| {
                        osm_node_to_graph_idx
                            .get(&node_id.0)
                            .copied()
                            .map(|index| (node_id.0, index))
                    })
                    .collect();
                for pair in way_nodes.windows(2) {
                    let (from_osm, from) = pair[0];
                    let (to_osm, to) = pair[1];
                    if from == to || Self::directed_edge_exists(&graph, from, to) {
                        continue;
                    }

                    let mut payload = EdgePL::new();
                    payload.geometry = LineString::new(vec![
                        graph.nodes[from].payload.point.into(),
                        graph.nodes[to].payload.point.into(),
                    ]);
                    payload.lines = transit_lines.clone();
                    payload.level = Self::routing_level(&way.tags, profile);
                    payload.oneway = Self::oneway(&way.tags, profile);
                    payload.preferred_direction = Self::parse_preferred_direction(&way.tags);
                    payload.allowed_modes = modes;
                    payload.osmid = way_id;
                    payload.restriction = restricted_ways.contains(&way_id)
                        && (restriction_via_nodes.contains(&from_osm)
                            || restriction_via_nodes.contains(&to_osm));
                    payload.cost = Self::cost_for_profile(payload.level, profile, payload.length());

                    graph.add_edge(from, to, payload);
                }
            }
        }

        // Upstream keeps relation maps only while constructing the graph. Route
        // identity has already been compiled into compact edge line IDs, so do
        // not retain relation node/edge coverage through matching.
        let relations_list: Vec<OsmRelation> = Vec::new();
        let final_node_to_rels: AHashMap<NodeIndex, Vec<usize>> = AHashMap::new();

        // Drop construction-only structures before allocating the compact graph.
        drop(way_transit_info);
        drop(osm_node_to_graph_idx);
        drop(ways_in_relations);
        drop(ways_in_ferry_relations);
        drop(kept_ways);
        drop(restricted_ways);
        drop(restriction_via_nodes);

        // C++ order here is collapse -> writeGeoms/costs -> components ->
        // geometry simplification -> opposite-direction edges -> one-way penalty.
        // Rust does not currently have upstream's 0.5m simplifier; this patch
        // leaves that pre-existing difference unchanged. Recompute
        // costs from the full collapsed geometry and profile level speed rather
        // than summing rounded two-point segment costs.
        collapse_edges(&mut graph, &mut graph_node_flags);
        for edge in &mut graph.edges {
            edge.payload.cost =
                Self::cost_for_profile(edge.payload.level, profile, edge.payload.length());
        }
        write_components(&mut graph);
        write_other_direction_edges(&mut graph);
        apply_one_way_penalty(
            &mut graph,
            profile.one_way_penalty_factor(),
            profile.one_way_entry_cost_seconds(),
        );

        // Full matching builds a single edge R-tree directly from collapsed
        // polyline geometry. The legacy node R-tree and retained relation maps
        // are intentionally empty to avoid duplicate indexing/metadata.
        let spatial_tree = None;

        println!(
            "Graph built: {} nodes, {} directed edges, {} canonical transit lines, {} relations",
            graph.nodes.len(),
            graph.edges.len(),
            graph.transit_lines.len(),
            relations_list.len()
        );

        Ok(OsmData {
            graph,
            timestamp: "unknown".to_string(),
            spatial_tree,
            osm_filepath: path.to_path_buf(),
            relations: relations_list,
            node_to_relations: final_node_to_rels,
        })
    }

    fn transit_info_for_relation(relation: &PreRelation) -> Option<TransitInfo> {
        let short_name = relation
            .tags
            .get("ref")
            .or_else(|| relation.tags.get("name"))
            .map(|value| value.to_string())
            .unwrap_or_default();
        let from_str = relation
            .tags
            .get("from")
            .map(|value| value.to_string())
            .unwrap_or_default();
        let to_str = relation
            .tags
            .get("to")
            .map(|value| value.to_string())
            .unwrap_or_default();
        if short_name.is_empty() && from_str.is_empty() && to_str.is_empty() {
            None
        } else {
            Some(TransitInfo {
                short_name,
                from_str,
                to_str,
            })
        }
    }

    fn directed_edge_exists(graph: &Graph<NodePL, EdgePL>, from: NodeIndex, to: NodeIndex) -> bool {
        graph.nodes[from]
            .out_edges
            .iter()
            .any(|&edge_idx| graph.edges[edge_idx].to == to)
    }

    fn is_route_geometry_role(role: &str) -> bool {
        matches!(role, "" | "forward" | "backward")
    }

    pub fn is_valid_way(way: &osmpbfreader::Way) -> bool {
        way.nodes.len() > 1
    }

    pub fn is_infrastructure(way: &osmpbfreader::Way) -> bool {
        if Self::is_platform(way) {
            return false;
        }
        way.tags.get("route").is_some_and(|route| route == "ferry")
            || way.tags.contains_key("aerialway")
            || way.tags.contains_key("railway")
            || way.tags.contains_key("highway")
    }

    pub fn is_platform(way: &osmpbfreader::Way) -> bool {
        matches!(
            way.tags.get("railway").map(|value| value.as_str()),
            Some("platform" | "stop" | "platform_edge")
        ) || matches!(
            way.tags.get("public_transport").map(|value| value.as_str()),
            Some("platform" | "stop_position" | "station")
        )
    }

    fn node_flags_for_profile(tags: &Tags, profile: OsmProfile) -> u8 {
        if profile == OsmProfile::Bus {
            return node_flags(tags);
        }

        let railway = tags.get("railway").map(|value| value.as_str());
        let station = match profile {
            OsmProfile::Rail => {
                tags.get("public_transport").map(|v| v.as_str()) == Some("stop_position")
                    || matches!(railway, Some("stop" | "halt" | "station" | "subway_stop"))
                    || tags.contains_key("tram_stop")
                    || tags.contains_key("stop")
            }
            OsmProfile::TramSubway | OsmProfile::Funicular => {
                tags.get("public_transport").map(|v| v.as_str()) == Some("stop_position")
                    || matches!(
                        tags.get("station").map(|v| v.as_str()),
                        Some("subway" | "tram")
                    )
                    || matches!(
                        railway,
                        Some("stop" | "halt" | "station" | "tram_stop" | "subway_stop")
                    )
                    || tags.contains_key("tram_stop")
                    || tags.contains_key("stop")
            }
            OsmProfile::Gondola => {
                matches!(
                    tags.get("aerialway").map(|v| v.as_str()),
                    Some("station" | "stop")
                ) || tags.get("public_transport").map(|v| v.as_str()) == Some("stop_position")
                    || matches!(
                        tags.get("station").map(|v| v.as_str()),
                        Some("subway" | "tram")
                    )
                    || matches!(
                        railway,
                        Some("stop" | "halt" | "station" | "tram_stop" | "subway_stop")
                    )
                    || tags.contains_key("tram_stop")
                    || tags.contains_key("stop")
            }
            OsmProfile::Ferry => {
                tags.get("ferry").map(|v| v.as_str()) == Some("yes")
                    || tags.get("public_transport").map(|v| v.as_str()) == Some("stop_position")
                    || tags.get("amenity").map(|v| v.as_str()) == Some("ferry_terminal")
                    || tags.get("mooring").map(|v| v.as_str()) == Some("ferry")
                    || tags.get("station").map(|v| v.as_str()) == Some("ferry")
                    || matches!(railway, Some("stop" | "halt" | "station"))
                    || tags.contains_key("stop")
            }
            OsmProfile::Generic(_) => node_flags(tags) & NODE_FLAG_STATION != 0,
            OsmProfile::Bus => unreachable!(),
        };

        if station { NODE_FLAG_STATION } else { 0 }
    }

    fn way_dropped_for_profile(way: &osmpbfreader::Way, profile: OsmProfile) -> bool {
        let tags = &way.tags;
        let railway = tags.get("railway").map(|value| value.as_str());
        let highway = tags.get("highway").map(|value| value.as_str());

        match profile {
            OsmProfile::Rail => {
                matches!(
                    railway,
                    Some(
                        "abandoned"
                            | "construction"
                            | "disused"
                            | "miniature"
                            | "signal"
                            | "razed"
                            | "proposed"
                            | "platform"
                    )
                ) || tags.get("metro").map(|v| v.as_str()) == Some("yes")
                    || tags.get("area").map(|v| v.as_str()) == Some("yes")
                    || tags.get("type").map(|v| v.as_str()) == Some("multipolygon")
                    || tags.get("public_transport").map(|v| v.as_str()) == Some("platform")
                    || tags
                        .get("building")
                        .is_some_and(|v| v == "yes" || v == "train_station" || v == "roof")
                    || tags.get("amenity").map(|v| v.as_str()) == Some("shelter")
                    || tags.get("amenity").map(|v| v.as_str()) == Some("bus_station")
            }
            OsmProfile::TramSubway | OsmProfile::Gondola | OsmProfile::Funicular => {
                tags.get("area").map(|v| v.as_str()) == Some("yes")
                    || tags.get("public_transport").map(|v| v.as_str()) == Some("stop_area")
                    || tags.get("type").map(|v| v.as_str()) == Some("multipolygon")
                    || railway == Some("platform")
                    || tags.get("public_transport").map(|v| v.as_str()) == Some("platform")
                    || tags.get("service").map(|v| v.as_str()) == Some("alley")
            }
            OsmProfile::Bus => OsmFilter::bus_way_dropped(&way.tags),
            OsmProfile::Ferry => false,
            OsmProfile::Generic(_) => Self::is_platform(way),
        }
    }

    fn way_kept_for_profile(
        way: &osmpbfreader::Way,
        profile: OsmProfile,
        ferry_ways: &AHashSet<i64>,
    ) -> bool {
        if Self::way_dropped_for_profile(way, profile) {
            return false;
        }

        let tags = &way.tags;
        let railway = tags.get("railway").map(|value| value.as_str());
        let route = tags.get("route").map(|value| value.as_str());
        match profile {
            OsmProfile::Rail => {
                matches!(
                    railway,
                    Some("rail" | "light_rail" | "tram" | "narrow_gauge")
                ) || matches!(route, Some("rail" | "light_rail" | "train"))
            }
            OsmProfile::TramSubway => {
                matches!(route, Some("tram" | "funicular" | "subway" | "light_rail"))
                    || matches!(
                        railway,
                        Some(
                            "subway"
                                | "light_rail"
                                | "tram"
                                | "funicular"
                                | "station"
                                | "halt"
                                | "tram_stop"
                        )
                    )
                    || tags.get("subway").map(|v| v.as_str()) == Some("yes")
                    || tags.get("tram").map(|v| v.as_str()) == Some("yes")
            }
            OsmProfile::Bus => OsmFilter::bus_way_kept(&way.tags),
            OsmProfile::Ferry => {
                route == Some("ferry")
                    || ferry_ways.contains(&way.id.0)
                    || tags.get("waterway").map(|v| v.as_str()) == Some("river")
                    || tags.get("motorboat").map(|v| v.as_str()) == Some("yes")
                    || tags.get("ferry").map(|v| v.as_str()) == Some("yes")
                    || tags.get("amenity").map(|v| v.as_str()) == Some("ferry_terminal")
                    || tags.get("mooring").map(|v| v.as_str()) == Some("ferry")
                    || tags.get("station").map(|v| v.as_str()) == Some("ferry")
            }
            OsmProfile::Gondola => matches!(
                tags.get("aerialway").map(|v| v.as_str()),
                Some("gondola" | "cable_car" | "chair_lift" | "mixed_lift")
            ),
            OsmProfile::Funicular => {
                matches!(route, Some("funicular" | "tram" | "subway" | "light_rail"))
                    || matches!(
                        railway,
                        Some(
                            "funicular"
                                | "narrow_gauge"
                                | "subway"
                                | "light_rail"
                                | "tram"
                                | "station"
                                | "halt"
                                | "tram_stop"
                        )
                    )
                    || tags.get("subway").map(|v| v.as_str()) == Some("yes")
                    || tags.get("tram").map(|v| v.as_str()) == Some("yes")
            }
            OsmProfile::Generic(mask) => {
                Self::is_infrastructure(way) && (Self::way_mode_mask(way, ferry_ways) & mask) != 0
            }
        }
    }

    fn routing_level(tags: &Tags, profile: OsmProfile) -> i32 {
        let service = tags.get("service").map(|value| value.as_str());
        let usage = tags.get("usage").map(|value| value.as_str());
        let railway = tags.get("railway").map(|value| value.as_str());
        let route = tags.get("route").map(|value| value.as_str());
        let traffic_mode = tags.get("railway:traffic_mode").map(|value| value.as_str());

        match profile {
            OsmProfile::Rail => {
                // C++ OsmFilter::level scans from 0 upward, so lower numbered
                // matching levels take precedence when multiple tags match.
                if usage == Some("branch") {
                    1
                } else if railway == Some("tram") || service == Some("siding") {
                    2
                } else if matches!(service, Some("crossover" | "yard")) {
                    3
                } else if matches!(usage, Some("industrial" | "military" | "test"))
                    || service == Some("spur")
                    || traffic_mode == Some("freight")
                {
                    5
                } else {
                    0
                }
            }
            OsmProfile::TramSubway => {
                if service == Some("siding") {
                    2
                } else if railway == Some("funicular") || route == Some("funicular") {
                    3
                } else if matches!(service, Some("crossover" | "yard")) {
                    5
                } else {
                    0
                }
            }
            OsmProfile::Bus => crate::upstream_graph::bus_level(tags) as i32,
            OsmProfile::Ferry | OsmProfile::Gondola => 0,
            OsmProfile::Funicular => {
                if service == Some("siding") {
                    2
                } else if matches!(
                    route,
                    Some("tram" | "narrow_gauge" | "subway" | "light_rail")
                ) || matches!(
                    railway,
                    Some(
                        "narrow_gauge"
                            | "subway"
                            | "light_rail"
                            | "tram"
                            | "station"
                            | "halt"
                            | "tram_stop"
                    )
                ) || tags.get("subway").map(|v| v.as_str()) == Some("yes")
                    || tags.get("tram").map(|v| v.as_str()) == Some("yes")
                {
                    3
                } else if matches!(service, Some("crossover" | "yard")) {
                    5
                } else {
                    0
                }
            }
            OsmProfile::Generic(_) => Self::parse_level(tags),
        }
    }

    fn oneway(tags: &Tags, profile: OsmProfile) -> u8 {
        match profile {
            OsmProfile::Rail => {
                // Exact [rail] undirected filter: notably `oneway=-1` is
                // explicitly treated as bidirectional upstream.
                if matches!(
                    tags.get("oneway").map(|v| v.as_str()),
                    Some("false" | "no" | "-1")
                ) || tags.get("railway:preferred_direction").map(|v| v.as_str()) == Some("both")
                    || tags.get("railway:bidirectional").map(|v| v.as_str()) == Some("regular")
                {
                    0
                } else if tags.get("railway:preferred_direction").map(|v| v.as_str())
                    == Some("backward")
                {
                    2
                } else if tags.get("oneway").map(|v| v.as_str()) == Some("yes")
                    || tags.get("railway:preferred_direction").map(|v| v.as_str())
                        == Some("forward")
                {
                    1
                } else {
                    0
                }
            }
            OsmProfile::Bus => crate::upstream_graph::bus_oneway(tags),
            OsmProfile::TramSubway | OsmProfile::Gondola | OsmProfile::Funicular => {
                if tags.get("oneway").map(|v| v.as_str()) == Some("yes") {
                    1
                } else {
                    0
                }
            }
            OsmProfile::Ferry => 0,
            OsmProfile::Generic(_) => Self::parse_oneway(tags),
        }
    }

    fn speed_for_profile(level: i32, profile: OsmProfile) -> f64 {
        let table: &[f64; 8] = match profile {
            OsmProfile::Rail => &[120.0, 90.0, 65.0, 50.0, 30.0, 20.0, 10.0, 5.0],
            OsmProfile::TramSubway | OsmProfile::Gondola | OsmProfile::Funicular => {
                &[85.0, 70.0, 55.0, 40.0, 30.0, 20.0, 10.0, 5.0]
            }
            OsmProfile::Bus => &[85.0, 70.0, 55.0, 40.0, 30.0, 20.0, 10.0, 5.0],
            OsmProfile::Ferry => &[70.0, 60.0, 50.0, 35.0, 30.0, 25.0, 10.0, 5.0],
            OsmProfile::Generic(_) => &[85.0, 70.0, 55.0, 40.0, 30.0, 20.0, 10.0, 5.0],
        };
        table[level.clamp(0, 7) as usize] / 3.6
    }

    fn cost_for_profile(level: i32, profile: OsmProfile, length_m: f64) -> u32 {
        let speed = Self::speed_for_profile(level, profile).max(0.1);
        ((length_m / speed) * 10.0).min(u32::MAX as f64).ceil() as u32
    }

    fn way_mode_mask(way: &osmpbfreader::Way, ferry_ways: &AHashSet<i64>) -> u8 {
        let (rail, tram, metro, bus, ferry, gondola) = Self::classify_way(way, ferry_ways);
        (if rail { MODE_RAIL } else { 0 })
            | (if tram { MODE_TRAM } else { 0 })
            | (if metro { MODE_SUBWAY } else { 0 })
            | (if bus { MODE_BUS } else { 0 })
            | (if ferry { MODE_FERRY } else { 0 })
            | (if gondola { MODE_GONDOLA } else { 0 })
    }

    pub fn classify_way(
        way: &osmpbfreader::Way,
        ferry_ways: &AHashSet<i64>,
    ) -> (bool, bool, bool, bool, bool, bool) {
        let railway = way.tags.get("railway").map(|value| value.as_str());
        let highway = way.tags.get("highway").map(|value| value.as_str());
        let route = way.tags.get("route").map(|value| value.as_str());
        let aerialway = way.tags.get("aerialway").map(|value| value.as_str());

        let rail =
            railway.is_some_and(|value| matches!(value, "rail" | "light_rail" | "narrow_gauge"));
        let tram = railway == Some("tram");
        let metro = railway == Some("subway");
        let ferry = route == Some("ferry") || ferry_ways.contains(&way.id.0);
        let gondola = aerialway.is_some();
        let bus = !ferry && !gondola && OsmFilter::bus_way_kept(&way.tags);
        (rail, tram, metro, bus, ferry, gondola)
    }

    fn parse_level(tags: &Tags) -> i32 {
        let service = tags.get("service").map(|value| value.as_str());
        let usage = tags.get("usage").map(|value| value.as_str());
        let railway = tags.get("railway").map(|value| value.as_str());
        let traffic_mode = tags.get("railway:traffic_mode").map(|value| value.as_str());

        if matches!(usage, Some("industrial" | "military" | "test"))
            || service == Some("spur")
            || traffic_mode == Some("freight")
        {
            5
        } else if matches!(service, Some("crossover" | "yard")) {
            3
        } else if service == Some("siding") || railway == Some("tram") {
            2
        } else if usage == Some("branch") {
            1
        } else {
            0
        }
    }

    fn parse_oneway(tags: &Tags) -> u8 {
        if matches!(
            tags.get("oneway").map(|value| value.as_str()),
            Some("no" | "false")
        ) || tags.get("railway:preferred_direction").map(|v| v.as_str()) == Some("both")
            || tags.get("railway:bidirectional").map(|v| v.as_str()) == Some("regular")
        {
            return 0;
        }
        match tags.get("oneway").map(|value| value.as_str()) {
            Some("yes" | "true" | "1") => return 1,
            Some("-1") => return 2,
            _ => {}
        }
        match tags
            .get("railway:preferred_direction")
            .map(|value| value.as_str())
        {
            Some("forward") => 1,
            Some("backward") => 2,
            _ => 0,
        }
    }

    fn parse_preferred_direction(tags: &Tags) -> u8 {
        match tags
            .get("railway:preferred_direction")
            .map(|value| value.as_str())
        {
            Some("forward") => 1,
            Some("backward") => 2,
            _ => 0,
        }
    }

    fn get_speed(tags: &Tags) -> f64 {
        if let Some(maxspeed) = tags.get("maxspeed") {
            if let Ok(value) = maxspeed.parse::<f64>() {
                return value / 3.6;
            }
        }
        if let Some(highway) = tags.get("highway") {
            return match highway.as_str() {
                "motorway" => 100.0 / 3.6,
                "trunk" => 80.0 / 3.6,
                "primary" => 70.0 / 3.6,
                "secondary" => 60.0 / 3.6,
                "tertiary" => 50.0 / 3.6,
                "residential" => 30.0 / 3.6,
                "living_street" => 10.0 / 3.6,
                "footway" | "pedestrian" => 4.0 / 3.6,
                _ => 50.0 / 3.6,
            };
        }
        if let Some(railway) = tags.get("railway") {
            return match railway.as_str() {
                "rail" => 100.0 / 3.6,
                "tram" => 40.0 / 3.6,
                "subway" => 80.0 / 3.6,
                _ => 50.0 / 3.6,
            };
        }
        if tags.get("route").is_some_and(|route| route == "ferry") {
            return 15.0 / 3.6;
        }
        if tags.contains_key("aerialway") {
            return 15.0 / 3.6;
        }
        10.0
    }

    fn calculate_cost(tags: &Tags, length_m: f64) -> u32 {
        let railway = tags.get("railway").map(|value| value.as_str());
        let speed = if matches!(railway, Some("rail" | "light_rail" | "narrow_gauge")) {
            const RAIL_SPEED_KMH: [f64; 8] = [120.0, 90.0, 65.0, 50.0, 30.0, 20.0, 10.0, 5.0];
            RAIL_SPEED_KMH[Self::parse_level(tags).clamp(0, 7) as usize] / 3.6
        } else {
            Self::get_speed(tags)
        };
        ((length_m / speed.max(0.1)) * 10.0)
            .min(u32::MAX as f64)
            .ceil() as u32
    }

    pub fn read_relations_only(path: &Path) -> Result<LightOsmData> {
        println!("Reading OSM relations only (light pass) from {:?}...", path);
        let mut pbf = Self::open_pbf(path)?;
        let mut relations = Vec::new();
        let mut way_to_relations: AHashMap<i64, Vec<usize>> = AHashMap::new();

        for obj in pbf.iter() {
            let obj = obj.context("Error reading PBF object")?;
            let OsmObj::Relation(relation) = obj else {
                continue;
            };
            // This light pass exists only for the tiled [bus, coach] path.
            // Apply the same relation keep values as upstream instead of
            // retaining every transit route relation in the extract.
            if !matches!(
                relation.tags.get("route").map(|value| value.as_str()),
                Some("bus" | "trolleybus")
            ) {
                continue;
            }

            let mut way_ids = Vec::new();
            for member in &relation.refs {
                if let OsmId::Way(way_id) = member.member {
                    way_ids.push(way_id.0);
                }
            }

            let relation_idx = relations.len();
            for &way_id in &way_ids {
                way_to_relations
                    .entry(way_id)
                    .or_default()
                    .push(relation_idx);
            }

            relations.push(LightRelation {
                ref_tag: relation.tags.get("ref").map(|value| value.to_string()),
                name: relation.tags.get("name").map(|value| value.to_string()),
                from_str: relation
                    .tags
                    .get("from")
                    .map(|value| value.to_string())
                    .unwrap_or_default(),
                to_str: relation
                    .tags
                    .get("to")
                    .map(|value| value.to_string())
                    .unwrap_or_default(),
                operator: relation.tags.get("operator").map(|value| value.to_string()),
                colour: relation
                    .tags
                    .get("colour")
                    .or_else(|| relation.tags.get("color"))
                    .map(|value| value.to_string()),
            });
        }

        println!("Loaded {} relations for light matching.", relations.len());
        Ok(LightOsmData {
            relations,
            way_to_relations,
        })
    }
}

pub fn load_osm(
    path: &Path,
    used_route_types: &AHashSet<RouteType>,
    bbox: Option<(f64, f64, f64, f64)>,
    skip_small_roads: bool,
) -> Result<OsmData> {
    OsmBuilder::read(path, used_route_types, bbox, skip_small_roads)
}
