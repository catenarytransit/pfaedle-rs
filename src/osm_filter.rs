//! OSM keep/drop filtering.
//!
//! Upstream pfaedle keeps OSM filtering separate from graph construction in
//! `osm/OsmFilter.{h,cpp}`. Keep the Rust port organized the same way: this
//! module decides whether an OSM entity is admissible, while `OsmBuilder`
//! remains responsible for turning admitted entities into graph edges.

use osmpbfreader::Tags;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum AccessDecision {
    Allow,
    Deny,
    Unspecified,
}

pub(crate) struct OsmFilter;

impl OsmFilter {
    fn access_decision(tags: &Tags, key: &str) -> AccessDecision {
        let Some(value) = tags.get(key).map(|value| value.as_str()) else {
            return AccessDecision::Unspecified;
        };

        match value {
            "no" => AccessDecision::Deny,

            // These values still describe legal/restricted access. They may be
            // assigned a poor routing level elsewhere, but they are not a hard
            // prohibition. `bus`/`psv` are also used as access values in OSM.
            "yes" | "designated" | "permissive" | "private" | "destination" | "delivery"
            | "customers" | "permit" | "bus" | "psv" => AccessDecision::Allow,

            _ => AccessDecision::Unspecified,
        }
    }

    /// Resolve bus access using OSM's access-key specificity hierarchy.
    ///
    /// A more specific key overrides a less specific one, e.g. `bus=yes`
    /// overrides `motor_vehicle=no`, while `bus=no` overrides `access=yes`.
    /// Pedestrian tags such as `foot=designated` are intentionally irrelevant.
    fn bus_access(tags: &Tags) -> AccessDecision {
        for key in ["bus", "psv", "motor_vehicle", "vehicle", "access"] {
            let decision = Self::access_decision(tags, key);
            if decision != AccessDecision::Unspecified {
                return decision;
            }
        }
        AccessDecision::Unspecified
    }

    /// Way-level equivalent of the upstream `[bus, coach]` `osm_filter_drop`
    /// rules, with one additional correctness guard for OSM motor access.
    ///
    /// Upstream's generic filter does not understand the access hierarchy, so
    /// the Rust port applies it here before relation inheritance can admit a
    /// path that explicitly forbids buses/motor vehicles.
    pub(crate) fn bus_way_dropped(tags: &Tags) -> bool {
        if Self::bus_access(tags) == AccessDecision::Deny {
            return true;
        }

        let highway = tags.get("highway").map(|value| value.as_str());

        // `train=yes|no_match_ways` deliberately does not apply to ways.
        // `public_transport=stop_area|no_match_nds|no_match_rels` applies only
        // to ways. These are the same entity-specific rules as pfaedle.cfg.
        tags.get("area").map(|value| value.as_str()) == Some("yes")
            || tags.get("public_transport").map(|value| value.as_str()) == Some("stop_area")
            || tags.get("type").map(|value| value.as_str()) == Some("multipolygon")
            || matches!(
                tags.get("railway").map(|value| value.as_str()),
                Some("platform" | "station")
            )
            || matches!(highway, Some("proposed" | "footway" | "construction"))
            || tags
                .get("building")
                .is_some_and(|value| value == "yes" || value == "train_station")
            || tags
                .get("leisure")
                .is_some_and(|value| value == "garden" || value == "park")
    }

    /// Way-level equivalent of the upstream `[bus, coach]` `osm_filter_keep`
    /// rules. Drop rules always win, exactly as in C++ `OsmBuilder::keepWay()`.
    pub(crate) fn bus_way_kept(tags: &Tags) -> bool {
        if Self::bus_way_dropped(tags) {
            return false;
        }

        let highway = tags.get("highway").map(|value| value.as_str());
        if matches!(
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
        ) {
            return true;
        }

        let any_value = |key: &str| tags.contains_key(key);
        let one_of = |key: &str, values: &[&str]| {
            tags.get(key)
                .is_some_and(|value| values.iter().any(|candidate| value.as_str() == *candidate))
        };

        matches!(
            tags.get("way").map(|value| value.as_str()),
            Some("primary" | "seconday" | "bus_guideway")
        ) || any_value("busway")
            || one_of("psv", &["yes", "designated"])
            || one_of("bus", &["yes", "designated"])
            || one_of("minibus", &["yes", "designated"])
            || one_of("trolley_wire", &["yes"])
            || one_of("trolleywire", &["yes"])
            || one_of("trolleybus", &["yes"])
            || one_of("trolley_bus", &["yes"])
            || matches!(
                tags.get("route").map(|value| value.as_str()),
                Some("bus" | "trolleybus")
            )
            || one_of("bus:lanes", &["yes", "designated", "1"])
            || one_of("lanes:bus", &["1", "2", "3"])
            || one_of("lanes:psv", &["1", "2", "3"])
            || any_value("bus_stop")
            || any_value("stop")
            || tags.get("public_transport").map(|value| value.as_str()) == Some("stop_position")
            || highway == Some("bus_stop")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tags(values: &[(&str, &str)]) -> Tags {
        let mut tags = Tags::new();
        for &(key, value) in values {
            tags.insert(key.into(), value.into());
        }
        tags
    }

    #[test]
    fn rejects_motor_vehicle_no_on_normal_bus_road() {
        let tags = tags(&[("highway", "residential"), ("motor_vehicle", "no")]);
        assert!(OsmFilter::bus_way_dropped(&tags));
        assert!(!OsmFilter::bus_way_kept(&tags));
    }

    #[test]
    fn rejects_relation_only_path_when_motor_vehicles_are_forbidden() {
        let tags = tags(&[
            ("highway", "path"),
            ("foot", "designated"),
            ("motor_vehicle", "no"),
        ]);
        assert!(OsmFilter::bus_way_dropped(&tags));
    }

    #[test]
    fn bus_permission_overrides_general_motor_vehicle_prohibition() {
        let tags = tags(&[
            ("highway", "residential"),
            ("motor_vehicle", "no"),
            ("bus", "yes"),
        ]);
        assert!(!OsmFilter::bus_way_dropped(&tags));
        assert!(OsmFilter::bus_way_kept(&tags));
    }

    #[test]
    fn psv_permission_overrides_vehicle_prohibition() {
        let tags = tags(&[("vehicle", "no"), ("psv", "designated")]);
        assert!(!OsmFilter::bus_way_dropped(&tags));
        assert!(OsmFilter::bus_way_kept(&tags));
    }

    #[test]
    fn motor_vehicle_permission_overrides_access_no() {
        let tags = tags(&[
            ("highway", "residential"),
            ("access", "no"),
            ("motor_vehicle", "yes"),
        ]);
        assert!(!OsmFilter::bus_way_dropped(&tags));
        assert!(OsmFilter::bus_way_kept(&tags));
    }

    #[test]
    fn bus_no_overrides_less_specific_psv_and_vehicle_permissions() {
        let tags = tags(&[
            ("highway", "residential"),
            ("vehicle", "yes"),
            ("psv", "yes"),
            ("bus", "no"),
        ]);
        assert!(OsmFilter::bus_way_dropped(&tags));
    }

    #[test]
    fn upstream_footway_drop_still_wins_over_bus_yes() {
        let tags = tags(&[("highway", "footway"), ("bus", "yes")]);
        assert!(OsmFilter::bus_way_dropped(&tags));
        assert!(!OsmFilter::bus_way_kept(&tags));
    }

    #[test]
    fn private_access_remains_routable_for_transit_matching() {
        let tags = tags(&[("highway", "residential"), ("access", "private")]);
        assert!(!OsmFilter::bus_way_dropped(&tags));
        assert!(OsmFilter::bus_way_kept(&tags));
    }
}
