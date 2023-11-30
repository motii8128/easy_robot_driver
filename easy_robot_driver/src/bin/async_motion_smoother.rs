use safe_drive::{
    context::Context,
    error::DynError,
    pr_info,
    logger::Logger,
    msg::common_interfaces::std_msgs,
};

use safe_drive::{
    topic::publisher::Publisher,
    topic::subscriber::Subscriber,
};

use async_std;
use ros2_rust_util::get_f64_parameter;

#[async_std::main]
async fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("async_motion_smoother", None, Default::default())?;
    let subscriber = node.create_subscriber::<std_msgs::msg::Float32>("/input", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::Float32>("/output", None)?;

    let smooth_gain = get_f64_parameter(node.get_name(), "gain", 0.1) as f32;
    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    let smooth_task = async_std::task::spawn(async_smoother(subscriber, publisher, smooth_gain));

    smooth_task.await?;

    pr_info!(log, "Shutdown {}", node.get_name());    
    Ok(())
}

pub async fn async_smoother(
    mut subscriber:Subscriber<std_msgs::msg::Float32>,
    publisher:Publisher<std_msgs::msg::Float32>,
    gain:f32
)->Result<(), DynError>
{
    loop {
        let msg = subscriber.recv().await?;

        let target_ = msg.data;
        let mut history = 0.0;

        while history < target_
        {
            let mut send_msg = std_msgs::msg::Float32::new().unwrap();

            history += gain;
            
            send_msg.data = history;
            let _ = publisher.send(&send_msg);
        }
    }
}
