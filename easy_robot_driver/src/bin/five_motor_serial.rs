use safe_drive::{
    context::Context,
    error::DynError,
    pr_info,
    logger::Logger,
    msg::common_interfaces::std_msgs,
};

use async_std;
use easy_robot_driver;
use ros2_rust_util::{get_str_parameter, get_i64_parameter};

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("five_motor_serial", None, Default::default())?;

    let subscriber_fl = node.create_subscriber::<std_msgs::msg::Float32>("/motor/1", None)?;
    let subscriber_fr = node.create_subscriber::<std_msgs::msg::Float32>("/motor/2", None)?;
    let subscriber_rl = node.create_subscriber::<std_msgs::msg::Float32>("/motor/3", None)?;
    let subscriber_rr = node.create_subscriber::<std_msgs::msg::Float32>("/motor/4", None)?;
    let subscriber_ex = node.create_subscriber::<std_msgs::msg::Float32>("/motor/5", None)?;

    let port_name = get_str_parameter(node.get_name(), "port_name", "dev/ttyACM0");
    let baud_rate = get_i64_parameter(node.get_name(), "baudrate", 115200) as u32;

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    let writer = async_std::task::spawn(easy_robot_driver::five_motor_serial_writer(
        subscriber_fl, 
        subscriber_fr, 
        subscriber_rl, 
        subscriber_rr, 
        subscriber_ex, 
        port_name, 
        baud_rate));

    writer.await?;

    pr_info!(log, "Shutdown {}", node.get_name());    
    Ok(())
}