use safe_drive::{
    topic::{subscriber::Subscriber, publisher::Publisher},
    msg::common_interfaces::{geometry_msgs, sensor_msgs}, 
    error::DynError,
    logger::Logger,
    pr_info,
    clock::Clock,
};

extern crate nalgebra as na;

pub async fn async_pose_to_twist(
    mut subscriber:Subscriber<geometry_msgs::msg::Pose>,
    publisher:Publisher<geometry_msgs::msg::Twist>,
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    let mut clock = Clock::new().unwrap();
    let mut t_a = clock.get_now().unwrap() as f64;
    let mut old_pose = geometry_msgs::msg::Pose::new().unwrap();
    let mut send_msg = geometry_msgs::msg::Twist::new().unwrap();

    loop
    {
        let msg = subscriber.recv().await?;
        let t_b = clock.get_now().unwrap() as f64;
        let delta_t= (t_b - t_a)*10e-10;
        send_msg.linear.x = (msg.position.x - old_pose.position.x) / delta_t;
        send_msg.linear.y = (msg.position.y - old_pose.position.y) / delta_t;
        send_msg.linear.z = (msg.position.z - old_pose.position.z) / delta_t;

        let _ = publisher.send(&send_msg)?;
        pr_info!(log, "calc linear_x:{}, linear_y:{}, linear_z:{}", send_msg.linear.x, send_msg.linear.y, send_msg.linear.z);

        old_pose.position.x = msg.position.x;
        old_pose.position.y = msg.position.y;
        old_pose.position.z = msg.position.z;

        t_a = t_b;
    }
}

pub async fn async_imu_to_vel(
    mut subscriber:Subscriber<sensor_msgs::msg::Imu>,
    publisher:Publisher<geometry_msgs::msg::Twist>
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());

    let mut clock = Clock::new().unwrap();
    let mut t_a = clock.get_now().unwrap() as f64;

    let mut old_imu = sensor_msgs::msg::Imu::new().unwrap();
    let mut send_msg = geometry_msgs::msg::Twist::new().unwrap();

    loop {
        let msg = subscriber.recv().await?;
        let t_b = clock.get_now().unwrap() as f64;
        let delta_t = (t_b - t_a)*10e-10;

        send_msg.linear.x = (msg.linear_acceleration.x - old_imu.linear_acceleration.x)*delta_t;
        send_msg.linear.y = (msg.linear_acceleration.y - old_imu.linear_acceleration.y)*delta_t;
        send_msg.linear.z = (msg.linear_acceleration.z - old_imu.linear_acceleration.z)*delta_t;

        let _ = publisher.send(&send_msg)?;
        pr_info!(log, "calc linear_x:{}, linear_y:{}, linear_z:{}", send_msg.linear.x, send_msg.linear.y, send_msg.linear.z);

        old_imu.linear_acceleration.x = msg.linear_acceleration.x;
        old_imu.linear_acceleration.y = msg.linear_acceleration.y;
        old_imu.linear_acceleration.z = msg.linear_acceleration.z;

        t_a = t_b;
    }
}

pub async fn async_twist_to_pose(
    mut subscriber:Subscriber<geometry_msgs::msg::Twist>,
    publisher:Publisher<geometry_msgs::msg::Pose>
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());

    let mut clock = Clock::new().unwrap();
    let mut t_a = clock.get_now().unwrap() as f64;

    let mut old_vel = geometry_msgs::msg::Twist::new().unwrap();
    let mut send_msg = geometry_msgs::msg::Pose::new().unwrap();

    loop {
        let msg = subscriber.recv().await?;
        let t_b = clock.get_now().unwrap() as f64;
        let delta_t = (t_b - t_a) * 10e-10;

        send_msg.position.x = (msg.linear.x - old_vel.linear.x)*delta_t;
        send_msg.position.y = (msg.linear.y - old_vel.linear.y)*delta_t;
        send_msg.position.z = (msg.linear.z - old_vel.linear.z)*delta_t;

        let _ = publisher.send(&send_msg)?;
        pr_info!(log, "Get pose x:{}, pose y:{}, pose z:{}", send_msg.position.x, send_msg.position.y, send_msg.position.z);

        old_vel.linear.x = msg.linear.x;
        old_vel.linear.y = msg.linear.y;
        old_vel.linear.z = msg.linear.z;

        t_a = t_b;
    }
}