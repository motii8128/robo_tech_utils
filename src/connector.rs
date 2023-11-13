use safe_drive::{
    error::DynError,
    logger::Logger,
    topic::publisher::Publisher,
    pr_info,
    pr_error,
    msg::common_interfaces::std_msgs,
    msg::U8Seq,
};

use async_net::UdpSocket;
use std::ptr;
use serde::{Deserialize, Serialize};

#[derive(Deserialize, Serialize)]
struct Msg
{
    data:f32
}

pub async fn udp_reciever(
    addr:&str,
    mut publisher:Publisher<std_msgs::msg::Float32>,
)->Result<(), DynError>
{
    let log = Logger::new(publisher.get_topic_name());
    pr_info!(log, "Start UDP_reciever({})", publisher.get_topic_name());

    let socket = UdpSocket::bind(addr).await?;

    let mut buf = [0; 1024];

    loop {
        if let Ok((size, src)) = socket.recv_from(&mut buf).await
        {
            let data:Result<Msg, _> = serde_json::from_slice(&buf[..size]);
        }
    }
}

fn deserialize<T:Sized>(buf: &[u8])->T
{
    let result_ptr = buf.as_ptr() as *const T;
    unsafe{ptr::read(result_ptr)}
}