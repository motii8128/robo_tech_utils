extern crate nalgebra as na;

pub mod ekf_posture;
pub mod pose;

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