use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
pub struct _Vector3_
{
     pub x:f64,
     pub y:f64,
     pub z:f64,
}

#[derive(Deserialize, Serialize)]
pub struct _Float32_
{
     pub data:f32
}

#[derive(Deserialize, Serialize)]
pub struct _Twist_
{
     pub linear:_Vector3_,
     pub angular:_Vector3_,
}

#[derive(Deserialize, Serialize)]
pub struct _Imu_
{
     pub linear_accel:_Vector3_,
     pub angular_velocity:_Vector3_,
}