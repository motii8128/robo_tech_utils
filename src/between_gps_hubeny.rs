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
    let base_base_latitude_rad = base_latitude_deg.to_radians();
    let base_base_longitude_rad = base_longitude_deg.to_radians();

    let dev_base_latitude_rad = dev_latitude_deg.to_radians();
    let dev_base_longitude_rad = dev_longitude_deg.to_radians();

    // lat diff
    let lat_diff = dev_base_latitude_rad - base_base_latitude_rad;
    // lon diff
    let lon_diff = dev_base_longitude_rad - base_base_longitude_rad;
    // average lat
    let lat_ave = (base_base_latitude_rad + dev_base_latitude_rad) / 2.0;


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

// get pose from gpsで算出した距離、方向から検算
pub fn verify_dev(
    base_latitude_deg:f64,
    base_longitude_deg:f64,
    distance:f64, 
    direction_deg:f64
)->na::Vector2<f64>
{
    let base_latitude_rad = base_latitude_deg.to_radians();
    let base_longitude_rad = base_longitude_deg.to_radians();
    let direcition_rad = direction_deg.to_radians();

    let second_eccentricity = (EQUATOR_RADIUS.powi(2) - POLAR_RADIUS.powi(2)) / EQUATOR_RADIUS.powi(2);
    let meridian_radius_curve = EQUATOR_RADIUS * 1e-2;

    let wt = (1.0 - second_eccentricity * (base_latitude_rad.sin()).powi(2)).sqrt();
    let mt = meridian_radius_curve / wt.powi(3);

    // 仮   
    let rad_diff_between_base_and_t = distance * (direcition_rad.cos() / mt);

    let rad_lat_ave = (base_latitude_rad + rad_diff_between_base_and_t) / 2.0;

    let w = (1.0 - second_eccentricity * (rad_lat_ave.sin()).powi(2)).sqrt();
    let m = meridian_radius_curve / (w.powi(3));
    let n = EQUATOR_RADIUS / w;

    // 緯度差分
    let rad_lat_diff = distance * direcition_rad.cos() / m;
    let dev_lat_rad = base_latitude_rad + rad_lat_diff;
    let dev_lat_deg = dev_lat_rad.to_degrees();

    // 経度差分
    let rad_lon_diff = distance * direcition_rad.sin() / (n * rad_lat_ave.cos());
    let dev_lon_rad = base_longitude_rad + rad_lon_diff;
    let dev_lon_deg = dev_lon_rad.to_degrees();

    let dev_lat_lon = na::Vector2::<f64>::new(
        dev_lat_deg,
        dev_lon_deg
    );

    dev_lat_lon
}