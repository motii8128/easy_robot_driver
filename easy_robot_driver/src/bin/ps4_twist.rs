use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::common_interfaces::{sensor_msgs, geometry_msgs},
};

/*  
JOY_LEFT_X  0
JOY_LEFT_Y  1
JOY_RIGHT_X 3
JOY_RIGHT_Y 4
UP_AND_DOWN 7
LEFT_AND_RIGHT 6
L2          2
R2          5

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

    let node = ctx.create_node("ps4_twist", None, Default::default())?;
    let mut selector = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<sensor_msgs::msg::Joy>("/joy", None)?;
    let publisher = node.create_publisher::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg|{
            let mut send_msg = geometry_msgs::msg::Twist::new().unwrap();

            send_msg.linear.x = *msg.axes.as_slice().get(0).unwrap() as f64;
            send_msg.linear.y = *msg.axes.as_slice().get(1).unwrap() as f64;
            send_msg.angular.z = *msg.axes.as_slice().get(3).unwrap() as f64;

            let _ = publisher.send(&send_msg);
        })
    );

    let log = Logger::new(node.get_name());

    pr_info!(log, "Start {}", node.get_name());

    loop {
        selector.wait()?;
    }
}