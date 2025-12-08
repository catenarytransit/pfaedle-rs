use gtfs_structures::RouteType;

fn main() {
    println!("Type for 0: {:?}", RouteType::Tram);
    println!("Type for 3: {:?}", RouteType::Bus);
    println!("Type for 700: {:?}", RouteType::Other(700));
}
