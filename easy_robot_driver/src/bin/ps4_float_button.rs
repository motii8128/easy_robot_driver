use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::{sensor_msgs, std_msgs},
};

use ros2_rust_util::get_i64_parameter;

/* 
__AXES__
JOY_LEFT_X  0
JOY_LEFT_Y  1
JOY_RIGHT_X 3
JOY_RIGHT_Y 4
UP_AND_DOWN 7
LEFT_AND_RIGHT 6
L2          2
R2          5

__BUTTON__
CROSS       0
CIRCLE      1
TRIANGLE    2
BOX         3
L1          4
R1          5
SHARE       8
OPTION      9
PS          10
*/

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("ps4_float_button", None, Default::default())?;
    let mut selector = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("/joy", None)?;
    let publisher = node.create_publisher::<std_msgs::msg::Float32>("/output", None)?;

    let num = get_i64_parameter(node.get_name(), "assigned_num", 0) as usize;

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg|{
            let mut send_msg = std_msgs::msg::Float32::new().unwrap();

            send_msg.data = *msg.buttons.as_slice().get(num).unwrap() as f32;

            let _ = publisher.send(&send_msg);
        })
    );

    let log = Logger::new(node.get_name());

    pr_info!(log, "Start {}", node.get_name());

    loop {
        selector.wait()?;
    }
}