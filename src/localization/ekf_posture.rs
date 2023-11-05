extern crate nalgebra as na;

pub fn init_posture()->na::Vector3<f64>
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

pub fn ekf_x(
    x:na::Vector3<f64>,
    u:na::Vector3<f64>,
    z:na::Vector2<f64>,
    p:na::Matrix3<f64>,
    r:na::Matrix2<f64>,
    q:na::Matrix3<f64>,
)->na::Vector3<f64>
{
    // predict
    let f = calc_jacob(x, u);

    let predict_x = predict_x(x, u);

    let predict_cov = predict_cov_transition(p, f, q);


    // update
    let y_res = update_obs_res(z, predict_x);

    let s = update_s(predict_cov, r);

    let kalman_gain = update_kalman_gain(predict_cov, s);

    let update_x = update_x(predict_x, y_res, kalman_gain);

    update_x
}

pub fn ekf_cov(
    x:na::Vector3<f64>,
    u:na::Vector3<f64>,
    p:na::Matrix3<f64>,
    r:na::Matrix2<f64>,
    q:na::Matrix3<f64>,
)->na::Matrix3<f64>
{
    // predict
    let f = calc_jacob(x, u);

    let predict_cov = predict_cov_transition(p, f, q);

    // update
    let s = update_s(predict_cov, r);

    let kalman_gain = update_kalman_gain(predict_cov, s);

    let update_cov = update_cov_transition(predict_cov, kalman_gain);

    update_cov
}


fn calc_jacob(x:na::Vector3<f64>, u:na::Vector3<f64>)->na::Matrix3<f64>
{
    let roll_cos = (x.x).cos();
    let roll_sin = (x.x).cos();

    let pitch_cos = (x.y).cos();
    let pitch_sin = (x.y).sin();

    let f = na::Matrix3::<f64>::new(
        1.0+u.y*((roll_cos*pitch_sin) / pitch_cos) - u.z*((roll_sin*pitch_sin) / pitch_cos), u.y*(roll_sin / (pitch_cos.powi(2))) + u.z * (roll_cos / (pitch_cos.powi(2))), 0.0,
        -1.0*u.y*roll_sin - u.z*roll_cos, 1.0, 0.0,
        u.y*(roll_cos/pitch_cos) - u.z*(roll_sin / pitch_cos), u.y*((roll_sin*pitch_sin) / (pitch_cos.powi(2))) + u.z*((roll_cos*pitch_sin) / (pitch_cos.powi(2))), 1.0
    );

    f
}

fn predict_x(x:na::Vector3<f64>, u:na::Vector3<f64>)->na::Vector3<f64>
{
    let roll_cos = (x.x).cos();
    let roll_sin = (x.x).cos();

    let pitch_cos = (x.y).cos();
    let pitch_sin = (x.y).sin();

    let predict_roll = x.x + u.x + u.y*((roll_sin*pitch_sin) / pitch_cos) + u.z*((roll_cos*pitch_sin) / pitch_cos);
    let predict_pitch = x.y + u.y*roll_cos - u.z*roll_sin;
    let predict_yaw = x.z + u.y*(roll_sin / pitch_cos) + u.z*(roll_cos / pitch_cos);

    let predict_x = na::Vector3::<f64>::new(
        predict_roll,
        predict_pitch,
        predict_yaw,
    );

    predict_x
}

fn get_h()->na::Matrix2x3<f64>
{
    let h = na::Matrix2x3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
    );

    h
}

fn predict_cov_transition(p:na::Matrix3<f64>, jacob_f:na::Matrix3<f64>, gyro_noise:na::Matrix3<f64>)->na::Matrix3<f64>
{
    let trans_jacob = jacob_f.transpose();
    let predict_cov = jacob_f * p * trans_jacob + gyro_noise;

    predict_cov
}

fn update_obs_res(obs:na::Vector2<f64>, x:na::Vector3<f64>)->na::Vector2<f64>
{
    let h = get_h();

    let residual = obs - h * x;

    residual
}

fn update_s(cov:na::Matrix3<f64>, accel_noise:na::Matrix2<f64>)->na::Matrix2<f64>
{
    let h = get_h();
    let trans_h = h.transpose();

    let s = h * cov * trans_h + accel_noise;

    s
}

fn update_kalman_gain(cov:na::Matrix3<f64>, s:na::Matrix2<f64>)->na::Matrix3x2<f64>
{
    let h = get_h();
    let trans_h = h.transpose();

    let inverse_s = s.try_inverse().unwrap();

    let kalman_gain = cov * trans_h * inverse_s;

    kalman_gain
}

fn update_x(x:na::Vector3<f64>, residual:na::Vector2<f64>, kalman_gain:na::Matrix3x2<f64>)->na::Vector3<f64>
{
    let update_x = x + kalman_gain * residual;

    update_x
}

fn update_cov_transition(cov:na::Matrix3<f64>, kalman_gain:na::Matrix3x2<f64>)->na::Matrix3<f64>
{
    let i = na::Matrix3::<f64>::new(
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    );

    let h = get_h();

    let update_cov = (i - kalman_gain*h) * cov;

    update_cov
}