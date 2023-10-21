extern crate nalgebra as na;

// 2023/10/21
const EQUATOR_RADIUS :f64 = 6378.137 * 10e3;
const POLAR_RADIUS :f64 = 6356.752 * 10e3;

pub fn get_pose_from_gps(
    base_latitude_deg:f64,
    base_longitude_deg:f64,
    dev_latitude_deg:f64,
    dev_longitude_deg:f64,
)->na::Vector2<f64>
{
    // convert radian
    let base_latitude_rad = base_latitude_deg.to_radians();
    let base_longitude_rad = base_longitude_deg.to_radians();

    let dev_latitude_rad = dev_latitude_deg.to_radians();
    let dev_longitude_rad = dev_longitude_deg.to_radians();

    // lat diff
    let lat_diff = dev_latitude_rad - base_latitude_rad;
    // lon diff
    let lon_diff = dev_longitude_rad - base_longitude_rad;
    // average lat
    let lat_ave = (base_latitude_rad + dev_latitude_rad) / 2.0;


    let second_eccentricity = (EQUATOR_RADIUS.powi(2) - POLAR_RADIUS.powi(2)) / EQUATOR_RADIUS.powi(2);

    let meridian_radius_curve = EQUATOR_RADIUS * 1e-2;

    let sin_lat_ave = lat_ave.sin();

    let w = (1.0 - second_eccentricity * sin_lat_ave.powi(2)).sqrt();

    let m = meridian_radius_curve / w.powi(3);

    let n = EQUATOR_RADIUS / w;

    let x_distance = m * lat_diff;

    let y_distance = -1.0 * n * lat_ave.cos() * lon_diff;

    let pose = na::Vector2::<f64>::new(
        x_distance,
        y_distance,
    );

    pose
}