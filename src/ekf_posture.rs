extern crate nalgebra as na;

pub fn vector3init()->na::Vector3<f64>
{
    let init = na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    );

    init
}

pub fn p_init(delta_t:f64)->na::Matrix3<f64>
{
    let dt_pow = delta_t.powi(2);
    let init = na::Matrix3::<f64>::new(
        0.0174*dt_pow, 0.0, 0.0,
        0.0, 0.0174*dt_pow, 0.0,
        0.0, 0.0, 0.0174*dt_pow,
    );

    init
}

pub fn calc_input(gyro:na::Vector3<f64>, delta_t:f64)->na::Vector3<f64>
{
    let u = na::Vector3::<f64>::new(
        gyro.x * delta_t,
        gyro.y * delta_t,
        gyro.z * delta_t,
    );

    u
}

pub fn calc_observe(accel:na::Vector3<f64>)->na::Vector2<f64>
{
    let calc_x_comp = accel.y / accel.z;
    let calc_y_comp = (-1.0*accel.x) / (((accel.y.powi(2))+(accel.z.powi(2))).sqrt());

    let z = na::Vector2::<f64>::new(
        calc_x_comp.atan(),
        calc_y_comp,
    );

    z
}

pub fn get_gyro_noise(delta_t:f64)->na::Matrix3<f64>
{
    let diagonal_comp = 0.0174* (delta_t.powi(2));
    let q = na::Matrix3::<f64>::new(
        diagonal_comp, 0.0, 0.0,
        0.0, diagonal_comp, 0.0,
        0.0, 0.0, diagonal_comp
    );

    q
}

pub fn get_accel_noise(delta_t:f64)->na::Matrix2<f64>
{
    let diagonal_comp = delta_t.powi(2);

    let r = na::Matrix2::<f64>::new(
        diagonal_comp, 0.0,
        0.0, diagonal_comp
    );

    r
}
