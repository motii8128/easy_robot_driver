use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::geometry_msgs,
};

use ros2_rust_util::get_str_parameter;
use easy_robot_driver::*;
use async_std;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("twist_reciever", None, Default::default())?;

    let log = Logger::new(node.get_name());

    let publisher = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let reciever_addr = get_str_parameter(node.get_name(), "sender_addr", "127.0.0.1:8080");

    pr_info!(log, "Start {}", node.get_name());

    let reciever = async_std::task::spawn(udp_twist_reciever(reciever_addr, publisher));

    reciever.await?;

    pr_info!(log, "shutdown server");

    Ok(())
}

