use async_std::channel;
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
                let data = 
                match  {
                    
                }
            }
            Err(e)=>
            {
                pr_error!(log, "[{}] :{:?}", publisher.get_topic_name(), e);
            }
        }
    }
}

#[derive(Deserialize, Serialize)]
pub struct _Vector3_
{
    pub 
}