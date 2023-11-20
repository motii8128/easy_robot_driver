use async_std::net::*;

use serde::{Deserialize, Serialize};

use safe_drive::{
    error::DynError,
    topic::{publisher::Publisher, subscriber::Subscriber},
    msg::common_interfaces::{geometry_msgs, std_msgs},
    logger::Logger,
    pr_info,
    pr_error
};

pub async fn udp_twist_reciever(
    reciever_addr:String,
    publisher:Publisher<geometry_msgs::msg::Twist>
)->Result<(), DynError>
{
    let log = Logger::new(publisher.get_topic_name());
    pr_info!(log, "Start twist reciever({})", publisher.get_topic_name());

    let socket = UdpSocket::bind(reciever_addr).await?;

    let mut buf = [0; 2048];

    loop {
        match socket.recv_from(&mut buf).await {
            Ok((size, get_addr))=>{
                let data:Result<_Twist_, serde_json::error::Error> = serde_json::from_slice(&buf[..size]);
                match data {
                    Ok(desialized)=>{
                        let mut msg = geometry_msgs::msg::Twist::new().unwrap();
                        msg.linear.x = desialized.linear.x;
                        msg.linear.y = desialized.linear.y;
                        msg.linear.z = desialized.linear.z;
                        msg.angular.x = desialized.angular.x;
                        msg.angular.y = desialized.angular.y;
                        msg.angular.z = desialized.angular.z;

                        let _ = publisher.send(&msg);
                    }
                    Err(e)=>{
                        pr_error!(log, "[{}] :{:?}", get_addr, e);
                    }
                }
            }
            Err(e)=>
            {
                pr_error!(log, "[{}] :{:?}", publisher.get_topic_name(), e);
            }
        }
    }
}

pub async fn udp_twist_transporter(
    sender_addr:String,
    reciever_addr:String,
    mut subscriber:Subscriber<geometry_msgs::msg::Twist>
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    pr_info!(log, "Start twist transporter({})", subscriber.get_topic_name());

    let socket = UdpSocket::bind(sender_addr).await?;

    loop {
        let msg = subscriber.recv().await?;

        let linear_ = _Vector3_{
            x:msg.linear.x,
            y:msg.linear.y,
            z:msg.linear.z,
        };

        let angular_ = _Vector3_{
            x:msg.angular.x,
            y:msg.angular.y,
            z:msg.angular.z,
        };

        let send_data = _Twist_{
            linear:linear_,
            angular:angular_,
        };

        let serialized = serde_json::to_string(&send_data);
        match serialized {
            Ok(data)=>{
                socket.send_to(data.as_bytes(), reciever_addr.as_str()).await?;
            }
            Err(e)=>{
                pr_error!(log, "[{}] :{:?}", subscriber.get_topic_name(), e);
            }
        }
    }
}

pub async fn udp_f32_reciever(
    reciever_addr:String,
    publisher:Publisher<std_msgs::msg::Float32>
)->Result<(), DynError>
{
    let log = Logger::new(publisher.get_topic_name());
    pr_info!(log, "Start twist reciever({})", publisher.get_topic_name());

    let socket = UdpSocket::bind(reciever_addr).await?;

    let mut buf = [0; 2048];

    loop {
        match socket.recv_from(&mut buf).await {
            Ok((size, get_addr))=>{
                let data:Result<_Float32_, serde_json::error::Error> = serde_json::from_slice(&buf[..size]);
                match data {
                    Ok(desialized)=>{
                        let mut msg = std_msgs::msg::Float32::new().unwrap();
                        msg.data = desialized.data;

                        let _ = publisher.send(&msg);
                    }
                    Err(e)=>{
                        pr_error!(log, "[{}] :{:?}", get_addr, e);
                    }
                }
            }
            Err(e)=>
            {
                pr_error!(log, "[{}] :{:?}", publisher.get_topic_name(), e);
            }
        }
    }
}

pub async fn udp_f32_transporter(
    sender_addr:String,
    reciever_addr:String,
    mut subscriber:Subscriber<std_msgs::msg::Float32>
)->Result<(), DynError>
{
    let log = Logger::new(subscriber.get_topic_name());
    pr_info!(log, "Start twist transporter({})", subscriber.get_topic_name());

    let socket = UdpSocket::bind(sender_addr).await?;

    loop {
        let msg = subscriber.recv().await?;

        let send_data = _Float32_{data:msg.data};

        let serialized = serde_json::to_string(&send_data);
        match serialized {
            Ok(data)=>{
                socket.send_to(data.as_bytes(), reciever_addr.as_str()).await?;
            }
            Err(e)=>{
                pr_error!(log, "[{}] :{:?}", subscriber.get_topic_name(), e);
            }
        }
    }
}


#[derive(Deserialize, Serialize)]
pub struct _Vector3_
{
    pub x:f64,
    pub y:f64,
    pub z:f64,
}

#[derive(Deserialize, Serialize)]
pub struct _Twist_
{
    pub linear:_Vector3_,
    pub angular:_Vector3_,
}

#[derive(Deserialize, Serialize)]
pub struct _Float32_
{
    pub data:f32,
}