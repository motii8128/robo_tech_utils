extern crate nalgebra as na;

pub mod ekf_posture;
pub mod ekf_pose;
pub mod between_gps_hubeny;

pub fn get_vector3<T>(x:T, y:T, z:T)->na::Vector3<T>
{
    let m = na::Vector3::<T>::new(
        x,
        y,
        z,
    );

    m
}

pub fn vector3init()->na::Vector3<f64>
{
    let init = na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    );

    init
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