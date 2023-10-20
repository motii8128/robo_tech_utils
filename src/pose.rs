extern crate nalgebra as na;

fn get_h()->na::Matrix3x6<f64>
{
    let h = na::Matrix3x6::<f64>::new(
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    h
}

pub fn init_poses()->na::Vector6<f64>
{
    let init = na::Vector6::<f64>::new(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    );

    init
}

pub fn init_cov()->na::Matrix6<f64>
{
    let p = na::Matrix6::<f64>::new(
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    );

    p
}

fn system(dt:f64)->na::Matrix6<f64>
{
    let f = na::Matrix6::<f64>::new(
        1.0, 0.0, 0.0, dt, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, dt, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, dt,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    f
}

fn noise(dt:f64)->na::Matrix6x3<f64>
{
    let g = na::Matrix6x3::<f64>::new(
        dt.powi(2)/2.0, 0.0, 0.0,
        0.0, dt.powi(2)/2.0, 0.0,
        0.0, 0.0, dt.powi(2)/2.0,
        dt, 0.0, 0.0,
        0.0, dt, 0.0,
        0.0, 0.0, dt,
    );

    g
}

pub fn observation(est:na::Vector6<f64>, dt:f64)->na::Vector3<f64>
{
    let true_value = na::Vector3::<f64>::new(
        est.x,
        est.y,
        est.z,
    );
    let obs_noise = na::Vector3::<f64>::new(
        dt.powi(2),
        dt.powi(2),
        dt.powi(2),
    );

    let z = true_value + obs_noise;

    z

}

fn cov(dt:f64)->na::Matrix6<f64>
{
    let g = noise(dt);

    let transpose_g = na::Matrix3x6::<f64>::new(
        g.m11, g.m21, g.m31, g.m41, g.m51, g.m61,
        g.m12, g.m22, g.m32, g.m42, g.m52, g.m62,
        g.m13, g.m23, g.m33, g.m43, g.m53, g.m63,
    );

    let sigma = dt.powi(2);

    let q = sigma.powi(2) * g * transpose_g;

    q
}

pub fn predict_noise(p_noise:na::Matrix6<f64>, dt:f64)->na::Matrix6<f64>
{
    let f = system(dt);
    let transpose_f = na::Matrix6::<f64>::new(
        f.m11, f.m21, f.m31, f.m41, f.m51, f.m61,
        f.m12, f.m22, f.m32, f.m42, f.m52, f.m62,
        f.m13, f.m23, f.m33, f.m43, f.m53, f.m63,
        f.m14, f.m24, f.m34, f.m44, f.m54, f.m64,
        f.m15, f.m25, f.m35, f.m45, f.m55, f.m65,
        f.m16, f.m26, f.m36, f.m46, f.m56, f.m66,
    );

    let q = cov(dt);

    let p = f * p_noise * transpose_f + q;

    p
}

pub fn predict_true(true_value:na::Vector6<f64>, linear_accel:na::Vector3<f64>, dt:f64)->na::Vector6<f64>
{
    let g = noise(dt);
    let f = system(dt);

    f * true_value + g * linear_accel
}

fn cov_obs_residuals(dt:f64, p_noise:na::Matrix6<f64>)->na::Matrix3<f64>
{
    let r = na::Matrix3::<f64>::new(
        dt.powi(2), 0.0, 0.0, 
        0.0, dt.powi(2), 0.0, 
        0.0, 0.0, dt.powi(2),
    );

    let h = get_h();

    let transpose_h = na::Matrix6x3::<f64>::new(
        h.m11, h.m21, h.m31,
        h.m12, h.m22, h.m32,
        h.m13, h.m23, h.m33,
        h.m14, h.m24, h.m34,
        h.m15, h.m25, h.m35,
        h.m16, h.m26, h.m36,
    );
    let s = (h * p_noise) * transpose_h + r;

    s
}

pub fn opt_kalman_gain(p_noise:na::Matrix6<f64>, dt:f64)->na::Matrix6x3<f64>
{
    let s = cov_obs_residuals(dt, p_noise);

    let inverse_s = s.try_inverse().unwrap();

    let h = get_h();

    let transpose_h = na::Matrix6x3::<f64>::new(
        h.m11, h.m21, h.m31,
        h.m12, h.m22, h.m32,
        h.m13, h.m23, h.m33,
        h.m14, h.m24, h.m34,
        h.m15, h.m25, h.m35,
        h.m16, h.m26, h.m36,
    );

    let kalman_gain = p_noise * transpose_h * inverse_s;

    kalman_gain
}

pub fn correction_est(est:na::Vector6<f64>, kalman_gain:na::Matrix6x3<f64>, z:na::Vector3<f64>)->na::Vector6<f64>
{
    let h = get_h();
    est + kalman_gain*(z - h * est)
}

pub fn correction_noise(p_noise:na::Matrix6<f64>, kalman_gain:na::Matrix6x3<f64>, dt:f64)->na::Matrix6<f64>
{
    let identity = na::Matrix6::<f64>::new(
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    );

    let r = na::Matrix3::<f64>::new(
        dt.powi(2), 0.0, 0.0, 
        0.0, dt.powi(2), 0.0, 
        0.0, 0.0, dt.powi(2),
    );

    let h = get_h();

    let iden_k = identity - kalman_gain*h;

    iden_k * p_noise * iden_k.transpose() + kalman_gain* r * kalman_gain.transpose()
}
