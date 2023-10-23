use safe_drive::{
    topic::{subscriber::Subscriber, publisher::Publisher},
    msg::common_interfaces::geometry_msgs, 
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
        send_msg.linear.x = (msg.position.x - old_pose.position.x)*delta_t;
        send_msg.linear.y = (msg.position.y - old_pose.position.y)*delta_t;
        send_msg.linear.z = (msg.position.z - old_pose.position.z)*delta_t;

        let _ = publisher.send(&send_msg)?;
        pr_info!(log, "calc linear_x:{}, linear_y:{}, linear_z:{}", send_msg.linear.x, send_msg.linear.y, send_msg.linear.z);

        old_pose.position.x = msg.position.x;
        old_pose.position.y = msg.position.y;
        old_pose.position.z = msg.position.z;

        t_a = t_b;
    }
}