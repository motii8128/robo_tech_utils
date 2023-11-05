extern crate nalgebra as na;

use safe_drive::msg::common_interfaces::sensor_msgs;

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

pub fn update_homo_matrix()
{
    
}