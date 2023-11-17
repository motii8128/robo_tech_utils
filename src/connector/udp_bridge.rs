use safe_drive::{
    error::DynError,
    logger::Logger,
    topic::{publisher::Publisher, subscriber::Subscriber},
    pr_info,
    pr_error,
    msg::common_interfaces::{std_msgs, geometry_msgs},
};

use async_std::channel;
use async_std::prelude::*;
use async_net::UdpSocket;
use signal_hook::consts::signal::*;
use signal_hook_async_std::Signals;

use crate::connector::udp_msgs;



pub async fn udp_f32_reciever(
    addr:String,
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

pub async fn udp_f32_sender(
    sender_addr:String,
    reciever_addr:String,
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

        socket.send_to(serialized_data.as_bytes(), reciever_addr.as_str()).await?;

        if closer.try_recv() == Ok(true) {
            pr_info!(log, "UDP servise shutdown");
            return Ok(());
        }
    }
}

pub async fn udp_twist_reciever(
    addr:String,
    closer: channel::Receiver<bool>,
    publisher:Publisher<geometry_msgs::msg::Twist>,
)->Result<(), DynError>
{
    let log = Logger::new(publisher.get_topic_name());
    pr_info!(log, "Start UDP reciever({})", publisher.get_topic_name());

    let socket = UdpSocket::bind(addr).await?;

    let mut buf = [0; 1024];

    loop {
        if let Ok((size, src)) = socket.recv_from(&mut buf).await
        {
            let data:Result<udp_msgs::_Twist_, _> = serde_json::from_slice(&buf[..size]);

            match data {
                Ok(desialized_data)=>
                {
                    let mut msg = geometry_msgs::msg::Twist::new().unwrap();

                    msg.linear.x = desialized_data.linear.x;
                    msg.linear.y = desialized_data.linear.y;
                    msg.linear.z = desialized_data.linear.z;
                    msg.angular.x = desialized_data.angular.x;
                    msg.angular.y = desialized_data.angular.y;
                    msg.angular.z = desialized_data.angular.z;

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

pub async fn udp_twist_sender(
    sender_addr:String,
    reciever_addr:String,
    closer: channel::Receiver<bool>,
    mut subscriber:Subscriber<geometry_msgs::msg::Twist>,
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    pr_info!(log, "Start UDP sender({})", subscriber.get_topic_name());

    let socket = UdpSocket::bind(sender_addr).await?;

    loop {
        let msg = subscriber.recv().await?;

        let linear = udp_msgs::_Vector3_{
            x:msg.linear.x,
            y:msg.linear.y,
            z:msg.linear.z,
        };

        let angular = udp_msgs::_Vector3_{
            x:msg.angular.x,
            y:msg.angular.y,
            z:msg.angular.z,
        };

        let send_data = udp_msgs::_Twist_{
            linear:linear,
            angular:angular,
        };

        let serialized_data = serde_json::to_string(&send_data).unwrap();

        socket.send_to(serialized_data.as_bytes(), reciever_addr.as_str()).await?;

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