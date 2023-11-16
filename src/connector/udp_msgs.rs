use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
pub struct _Float32_
{
     pub data:f32
}

#[derive(Deserialize, Serialize)]
pub struct _Twist_
{
     pub linear_x:f32,
     pub linear_y:f32,
     pub linear_z:f32,
     pub angular_x:f32,
     pub angular_y:f32,
     pub angular_z:f32,
}

#[derive(Deserialize, Serialize)]
pub struct _Imu_
{
     pub linear_accel_x:f32,
     pub linear_accel_y:f32,
     pub linear_accel_z:f32,
     pub angular_velocity_x:f32,
     pub angular_velocity_y:f32,
     pub angular_velocity_z:f32,
}