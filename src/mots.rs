use ahash::AHashSet;
use gtfs_structures::RouteType;
use std::str::FromStr;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MotCategory {
    Tram,
    Subway,
    Rail,
    Bus,
    Ferry,
    CableCar,
    Gondola,
    Funicular,
    Coach,
    Trolleybus,
    Monorail,
    Air,
    Other,
}

impl FromStr for MotCategory {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let s = s.to_lowercase();
        match s.as_str() {
            "tram" | "streetcar" | "light_rail" | "lightrail" | "light-rail" => {
                Ok(MotCategory::Tram)
            }
            "subway" | "metro" => Ok(MotCategory::Subway),
            "rail" | "train" => Ok(MotCategory::Rail),
            "bus" => Ok(MotCategory::Bus),
            "ferry" | "boat" | "ship" => Ok(MotCategory::Ferry),
            "cablecar" | "cable_car" | "cable-car" => Ok(MotCategory::CableCar),
            "gondola" => Ok(MotCategory::Gondola),
            "funicular" => Ok(MotCategory::Funicular),
            "coach" => Ok(MotCategory::Coach),
            "trolley" | "trolleybus" | "trolley-bus" => Ok(MotCategory::Trolleybus),
            "monorail" | "mono-rail" => Ok(MotCategory::Monorail),
            "air" => Ok(MotCategory::Air),
            _ => Err(format!("Unknown MOT category: {}", s)),
        }
    }
}

pub fn get_categories_from_string(s: &str) -> Result<AHashSet<MotCategory>, String> {
    if s == "all" {
        return Ok(vec![
            MotCategory::Tram,
            MotCategory::Subway,
            MotCategory::Rail,
            MotCategory::Bus,
            MotCategory::Ferry,
            MotCategory::CableCar,
            MotCategory::Gondola,
            MotCategory::Funicular,
            MotCategory::Coach,
            MotCategory::Trolleybus,
            MotCategory::Monorail,
            MotCategory::Air,
            MotCategory::Other,
        ]
        .into_iter()
        .collect());
    }

    let mut categories = AHashSet::new();
    for part in s.split(',') {
        let part = part.trim();
        if part.is_empty() {
            continue;
        }
        categories.insert(MotCategory::from_str(part)?);
    }
    Ok(categories)
}

pub fn map_route_type_to_category(rt: RouteType) -> MotCategory {
    match rt {
        RouteType::Tramway => MotCategory::Tram,
        RouteType::Subway => MotCategory::Subway,
        RouteType::Rail => MotCategory::Rail,
        RouteType::Bus => MotCategory::Bus,
        RouteType::Ferry => MotCategory::Ferry,
        RouteType::CableCar => MotCategory::CableCar,
        RouteType::Gondola => MotCategory::Gondola,
        RouteType::Funicular => MotCategory::Funicular,
        RouteType::Coach => MotCategory::Coach,
        // Match other variants if they exist or fallback
        RouteType::Other(t) => map_extended_route_type(t.into()),
        _ => MotCategory::Other,
    }
}

/// Check if a route type should use tiled processing (bus-like modes only).
/// Trams are excluded as they use dedicated track infrastructure.
pub fn is_bus_like_route_type(rt: RouteType) -> bool {
    matches!(
        rt,
        RouteType::Bus | RouteType::Coach | RouteType::Other(11) | RouteType::Other(800)
    )
}

/// Check if a MOT category should use tiled processing.
pub fn is_bus_like(cat: MotCategory) -> bool {
    matches!(
        cat,
        MotCategory::Bus | MotCategory::Coach | MotCategory::Trolleybus
    )
}

fn map_extended_route_type(t: i32) -> MotCategory {
    match t {
        // Specific overrides (handle before ranges)
        116 => MotCategory::Funicular,
        405 => MotCategory::Monorail,

        // Rail
        2 | 100..=117 | 300 | 1503 | 1504 => MotCategory::Rail,

        // Bus
        // 700..717, 1500, 1501, 1505..1507
        700..=717 | 1500 | 1501 | 1505..=1507 => MotCategory::Bus,

        // Subway
        1 | 400..=404 | 500 | 600 => MotCategory::Subway,

        // Tram
        0 | 900..=906 => MotCategory::Tram,

        // Ferry
        4 | 1000..=1021 | 1200 | 1502 => MotCategory::Ferry,

        // Gondola
        6 | 1300 | 1301 | 1304 | 1305 | 1306 | 1307 | 1101..=1114 => MotCategory::Gondola,

        // Funicular
        7 | 1302 | 1303 | 1400 => MotCategory::Funicular,

        // Cable Car
        5 => MotCategory::CableCar,

        // Trolleybus
        11 | 800 => MotCategory::Trolleybus,

        // Monorail
        12 => MotCategory::Monorail,

        // Coach
        200..=209 => MotCategory::Coach,

        // Air
        1100 => MotCategory::Air,

        _ => MotCategory::Other,
    }
}
