extern crate nalgebra as na;

use safe_drive::msg::common_interfaces::{sensor_msgs, geometry_msgs::msg::Point32};

pub fn icp_matching(
    previous_point:sensor_msgs::msg::PointCloud,
    current_point:sensor_msgs::msg::PointCloud,
    eps:f32, //0.0001
    max_iter:i32 //100
)
{
    let d_err = std::f32::INFINITY;

    let pre_err = std::f32::INFINITY;

    let count = 0;

    while d_err >= eps {
        
    }
}

pub fn nearest_neighbor_association(
    previous_point: &[Point32], 
    current_point: &[Point32]
)
{
    let mut delta_point 
    for i in 0..previous_point.len()
    {
        let 
    }
}