use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::{sensor_msgs, nav_msgs}, 
    topic::{subscriber::Subscriber, publisher::Publisher}
};

use async_std;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("imu_localizer", None, Default::default())?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Imu>("/imu", None)?;
    let publisher = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    let task = async_std::task::spawn(imu_localizer(subscriber, publisher));

    task.await?;

    pr_info!(log, "Shutdown {}", node.get_name());
    Ok(())
}

async fn imu_localizer(
    mut subscriber:Subscriber<sensor_msgs::msg::Imu>,
    publisher:Publisher<nav_msgs::msg::Odometry>,
)->Result<(), DynError>
{
    let mut send_msg = nav_msgs::msg::Odometry::new().unwrap();

    let mut history_time = 0.0;
    loop {
        let mut get_msg = subscriber.recv().await?;

        if get_msg.linear_acceleration.x.abs() < 0.1
        {
            get_msg.linear_acceleration.x = 0.0;
        }
        if get_msg.linear_acceleration.y.abs() < 0.1
        {
            get_msg.linear_acceleration.y = 0.0;
        }
        if get_msg.linear_acceleration.z.abs() < 9.9
        {
            get_msg.linear_acceleration.z = 0.0;
        }

        let recently_time = get_msg.header.stamp.sec as f64 + get_msg.header.stamp.nanosec as f64 * 10e-6;

        let delta_time = recently_time - history_time;

        send_msg.twist.twist.linear.x = get_msg.linear_acceleration.x * delta_time;
        send_msg.twist.twist.linear.y = get_msg.linear_acceleration.y * delta_time;
        send_msg.twist.twist.linear.z = get_msg.linear_acceleration.z * delta_time;

        send_msg.pose.pose.position.x = send_msg.twist.twist.linear.x * delta_time;
        send_msg.pose.pose.position.y = send_msg.twist.twist.linear.y * delta_time;
        send_msg.pose.pose.position.z = send_msg.twist.twist.linear.z * delta_time;

        history_time = recently_time;

        let _ = publisher.send(&send_msg);
    }
}
