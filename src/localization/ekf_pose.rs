extern crate nalgebra as na;

// 上からx, y, thita
pub fn init_pose()->na::Vector3<f64>
{
    let x = na::Vector3::<f64>::new(
        0.0,
        0.0,
        0.0,
    );

    x
}
