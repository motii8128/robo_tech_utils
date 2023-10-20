extern crate nalgebra as na;

pub fn x_init()->na::Vector3<f64>
{
    let init = na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    );

    init
}

pub fn p_init(dt:f64)->na::Matrix3<f64>
{
    let dt_pow = dt.powi(2);
    let init = na::Matrix3::<f64>::new(
        0.0174*dt_pow, 0.0, 0.0,
        0.0, 0.0174*dt_pow, 0.0,
        0.0, 0.0, 0.0174*dt_pow,
    );

    init
}

pub fn calc_input(gyro:na::Vector3<f64>, dt:f64)->na::Vector3<f64>
{
    let u = na::Vector3::<f64>::new(
        gyro.x * dt,
        gyro.y * dt,
        gyro.z * dt,
    );

    u
}

pub fn calc_observe(accel:na::Vector3<f64>)->na::Vector2<f64>
{
    let z = na::Vector2::<f64>::new(
        (accel.y / accel.z).atan(),
        (-1.0*accel.x)/((accel.y.powi(2) + accel.z.powi(2)).sqrt())
    );

    z
}