use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    msg::common_interfaces::std_msgs,
    pr_info,
};

use ros2_rust_util::{get_bool_parameter, get_f64_parameter};

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("motor_controller", None, Default::default())?;
    let mut selector = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<std_msgs::msg::Float32>("/input", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::Float32>("/output", None)?;

    let motor_power_rate = get_f64_parameter(node.get_name(), "motor_power_rate", 1.0) as f32;
    let enable_reverse = get_bool_parameter(node.get_name(), "enable_reverse", false);

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());


    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg|
        {
            let mut send_msg = std_msgs::msg::Float32::new().unwrap();
            send_msg.data = msg.data * motor_power_rate;
            if enable_reverse
            {
                send_msg.data *= -1.0;
            }

            let _ = publisher.send(&send_msg);
        })
    );

    loop {
        selector.wait()?;
    }
}