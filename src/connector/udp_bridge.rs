use safe_drive::{
    error::DynError,
    logger::Logger,
    topic::{publisher::Publisher, subscriber::Subscriber},
    pr_info,
    pr_error,
    msg::common_interfaces::std_msgs,
};

use async_std::channel;
use async_std::prelude::*;
use async_net::UdpSocket;
use signal_hook::consts::signal::*;
use signal_hook_async_std::Signals;

use crate::connector::udp_msgs;



pub async fn udp_reciever(
    addr:&str,
    closer: channel::Receiver<bool>,
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
            let data:Result<udp_msgs::_Float32_, _> = serde_json::from_slice(&buf[..size]);

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

            if closer.try_recv() == Ok(true) {
                pr_info!(log, "UDP servise shutdown");
                return Ok(());
            }
        }
    }
}

pub async fn udp_sender(
    sender_addr:&str,
    reciever_addr:&str,
    closer: channel::Receiver<bool>,
    mut subscriber:Subscriber<std_msgs::msg::Float32>,
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    pr_info!(log, "Start UDP sender({})", subscriber.get_topic_name());

    let socket = UdpSocket::bind(sender_addr).await?;

    loop {
        let msg = subscriber.recv().await?;

        let send_data = udp_msgs::_Float32_{data : msg.data};

        let serialized_data = serde_json::to_string(&send_data).unwrap();

        socket.send_to(serialized_data.as_bytes(), reciever_addr).await?;

        if closer.try_recv() == Ok(true) {
            pr_info!(log, "UDP servise shutdown");
            return Ok(());
        }
    }
}

pub async fn get_signal(closer: channel::Sender<bool>) -> Result<(), DynError> {
    let signals = Signals::new(&[SIGHUP, SIGTERM, SIGINT, SIGQUIT])?;
    let mut signals = signals.fuse();
    loop {
        if let Some(signal) = signals.next().await {
            match signal {
                SIGTERM | SIGINT | SIGQUIT => {
                    // Shutdown the system;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    async_std::task::sleep(std::time::Duration::from_millis(100)).await;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    closer.send(true).await?;
                    return Ok(());
                }
                _ => unreachable!(),
            }
        }
    }
}