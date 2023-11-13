extern crate nalgebra as na;

pub mod localization;
pub mod navigation;
pub mod slam;
pub mod connector;

pub fn get_vector3<T>(x:T, y:T, z:T)->na::Vector3<T>
{
    let m = na::Vector3::<T>::new(
        x,
        y,
        z,
    );

    m
}

pub fn identity_matrix()->na::Matrix3<f64>
{
    let i = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0, 
        0.0, 1.0, 0.0, 
        0.0, 0.0, 1.0,
    );

    i
}

pub fn euler_to_quaternion(euler:na::Vector3<f64>)->na::Vector4<f64>
{
    let cos_a = (euler.x / 2.0).cos();
    let sin_a = (euler.x / 2.0).sin();

    let cos_b = (euler.y / 2.0).cos();
    let sin_b = (euler.y / 2.0).sin();

    let cos_r = (euler.z / 2.0).cos();
    let sin_r = (euler.z / 2.0).sin();

    let quaternion = na::Vector4::<f64>::new(
        cos_a*cos_b*cos_r - sin_a*sin_b*sin_r,
        sin_a*cos_b*cos_r + cos_a*sin_b*sin_r,
        cos_a*sin_b*cos_r - sin_a*cos_b*sin_r,
        cos_a*cos_b*sin_r + sin_a*sin_b*cos_r,
    );

    quaternion
}

pub fn quaternion_to_euler_xyz(quaternion:na::Vector4<f64>)->na::Vector3<f64>
{
    let q_x = quaternion.x;

    let q_y = quaternion.y;

    let q_z = quaternion.z;

    let q_w = quaternion.w;

    let theta_x = -1.0 * ((2.0 * q_y * q_z - 2.0 * q_x * q_w) / (2.0 * (q_w.powi(2)) + 2.0 * (q_z.powi(2)) - 1.0));

    let theta_y = 2.0 + q_x * q_z + 2.0 * q_y * q_w;

    let theta_z = -1.0 * ((2.0 * q_x * q_y - 2.0 * q_z * q_w) / (2.0 * (q_w.powi(2)) + 2.0 * (q_x.powi(2)) - 1.0));

    let vec = na::Vector3::<f64>::new(
        theta_x.atan(),
        theta_y.asin(),
        theta_z.atan(),
    );

    vec
}

pub fn quaternion_to_euler_xzy(quaternion:na::Vector4<f64>)->na::Vector3<f64>
{
    let q_x = quaternion.x;

    let q_y = quaternion.y;

    let q_z = quaternion.z;

    let q_w = quaternion.w;

    let theta_x = (2.0 * q_y * q_z + 2.0 * q_x * q_w) / (2.0 * (q_w.powi(2)) + 2.0 * (q_y.powi(2)) - 1.0);

    let theta_y = -1.0 * ((2.0 + q_y * q_z - 2.0 * q_x * q_w) / (2.0 * q_w.powi(2) + 2.0 * q_z.powi(2) - 1.0));

    let theta_z = -1.0 * (2.0 * q_x * q_y - 2.0 * q_z * q_w);

    let vec = na::Vector3::<f64>::new(
        theta_x.atan(),
        theta_y.atan(),
        theta_z.asin(),
    );

    vec
}