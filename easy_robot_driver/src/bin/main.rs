use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::{sensor_msgs, std_msgs}, 
    topic::{subscriber::Subscriber, publisher::Publisher}
};

use async_std;
use ros2_rust_util::get_f64_parameter;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("safe_obstacle_scan", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::LaserScan>("/scan", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::Bool>("/obstacle", None)?;

    let danger_dist = get_f64_parameter(node.get_name(), "danger_dist", 1.0) as f32;
    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    let task = async_std::task::spawn(obstacle_scanner(subscriber, publisher, danger_dist));

    task.await?;

    pr_info!(log, "Shutdown {}", node.get_name());
    Ok(())
}

async fn obstacle_scanner(
    mut subscriber:Subscriber<sensor_msgs::msg::LaserScan>,
    publisher:Publisher<std_msgs::msg::Bool>,
    danger_dist:f32
)->Result<(), DynError>
{
    let mut send_msg = std_msgs::msg::Bool::new().unwrap();
    loop {
        send_msg.data = false;

        let msg = subscriber.recv().await?;

        let range = msg.ranges.as_slice();
        let front_point = *range.get(msg.ranges.len() / 2).unwrap();

        if front_point < danger_dist
        {
            send_msg.data = true;
        }

        let _ = publisher.send(&send_msg);
    }
}