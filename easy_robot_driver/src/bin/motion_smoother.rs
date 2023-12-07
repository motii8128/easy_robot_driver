use safe_drive::{
    context::Context,
    error::DynError,
    pr_info,
    logger::Logger,
    msg::common_interfaces::std_msgs,
};

use ros2_rust_util::get_f64_parameter;

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("async_motion_smoother", None, Default::default())?;
    let mut selector = ctx.create_selector()?;
    
    let subscriber = node.create_subscriber::<std_msgs::msg::Float32>("/input", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::Float32>("/output", None)?;

    let smooth_gain = get_f64_parameter(node.get_name(), "gain", 0.1) as f32;
    let log = Logger::new(node.get_name());

    let mut history = 0.0;

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg|{
            let target = msg.data;
            let mut send_msg = std_msgs::msg::Float32::new().unwrap();

            let vec = target - history;
            if vec > smooth_gain
            {
                if target > history
                {
                    send_msg.data = history + smooth_gain;
                }
                else
                {
                    send_msg.data = history - smooth_gain;
                }
            }
            else
            {
                send_msg.data = vec;
            }

            history = send_msg.data;

            let _ = publisher.send(&send_msg);
        })
    );
    
    pr_info!(log, "Start {}", node.get_name());

    loop{
        selector.wait()?;
    }
}
