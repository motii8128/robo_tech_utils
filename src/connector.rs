use safe_drive::{
    error::DynError,
    logger::Logger,
    topic::{publisher::Publisher, subscriber::Subscriber},
    pr_info,
    pr_error,
    msg::common_interfaces::std_msgs,
};

use async_net::UdpSocket;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
struct Msg
{
    data:f32
}

pub async fn udp_reciever(
    addr:&str,
    publisher:Publisher<std_msgs::msg::Float32>,
)->Result<(), DynError>
{
    let log = Logger::new(publisher.get_topic_name());
    pr_info!(log, "Start UDP reciever({})", publisher.get_topic_name());

    let socket = UdpSocket::bind(addr).await?;

    let mut buf = [0; 1024];

    loop {
        if let Ok((size, src)) = socket.recv_from(&mut buf).await
        {
            let data:Result<Msg, _> = serde_json::from_slice(&buf[..size]);

            match data {
                Ok(desialized_data)=>
                {
                    let mut msg = std_msgs::msg::Float32::new().unwrap();

                    msg.data = desialized_data.data;

                    let _ = publisher.send(&msg);
                }
                Err(e)=>
                {
                    pr_error!(log, "{} error : {}", src, e);
                }
            }
        }
    }
}

pub async fn udp_sender(
    sender_addr:&str,
    reciever_addr:&str,
    mut subscriber:Subscriber<std_msgs::msg::Float32>,
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    pr_info!(log, "Start UDP sender({})", subscriber.get_topic_name());

    let socket = UdpSocket::bind(sender_addr).await?;

    loop {
        let msg = subscriber.recv().await?;

        let send_data = Msg{data : msg.data};

        let serialized_data = serde_json::to_string(&send_data).unwrap();

        socket.send_to(serialized_data.as_bytes(), reciever_addr).await?;
    }
}