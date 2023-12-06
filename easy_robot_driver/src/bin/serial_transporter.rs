use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    pr_error,
    msg::common_interfaces::std_msgs, topic::subscriber::Subscriber
};

use serialport::SerialPort;
use async_std;
use std::{io::prelude::*, time::Duration};
use ros2_rust_util::*;
#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("serial_transporter", None, Default::default())?;

    let subscriber = node.create_subscriber::<std_msgs::msg::UInt8MultiArray>("/input", None)?;

    let port_name_ = get_str_parameter(node.get_name(), "port_name", "/dev/ttyACM0");
    let baud_rate_ = get_i64_parameter(node.get_name(), "baud_rate", 115200) as u32;

    let serialport = serialport::new(port_name_, baud_rate_).timeout(Duration::from_millis(100)).open()?;

    let writer = async_std::task::spawn(writer(subscriber, serialport));

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    writer.await?;

    pr_info!(log, "Shutdown {}", node.get_name());
    Ok(())
}

async fn writer(
    mut subscriber:Subscriber<std_msgs::msg::UInt8MultiArray>,
    mut serialport: Box<dyn SerialPort>
)->Result<(), DynError>
{
    let logger = Logger::new("Serial Writer");
    loop {
        let msg = subscriber.recv().await?;

        match serialport.write(msg.data.as_slice()){
            Ok(_)=>{
                if let Err(e) = std::io::stdout().flush() {
                    pr_error!(logger, "Failed to flush stdout: {:?}" , e)
                }
            },
            Err(e)=>{
                pr_error!(logger, "{:?}", e);
            }
        }
    }
}
