use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::{common_interfaces::{geometry_msgs, nav_msgs}, RosString}
};

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;
    let node = ctx.create_node("command_visualizer", None, Default::default())?;
    let mut selector = ctx.create_selector()?;

    let subscriber = node.create_subscriber::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;
    let publisher = node.create_publisher::<nav_msgs::msg::Odometry>("/odom", None)?;

    let mut send_msg = nav_msgs::msg::Odometry::new().unwrap();
    let mut z = 0.0;
    send_msg.header.frame_id = RosString::new("map").unwrap();
    send_msg.child_frame_id = RosString::new("odom").unwrap();

    selector.add_subscriber(
        subscriber, 
        Box::new(move |msg|{
            send_msg.pose.pose.position.x += msg.linear.x;
            send_msg.pose.pose.position.y += msg.linear.y;
            z += msg.angular.z;
            
            let cos_a = 0.0_f64.cos();
            let cos_b = 0.0_f64.cos();
            let cos_r = z.cos();
            let sin_a = 0.0_f64.sin();
            let sin_b = 0.0_f64.sin();
            let sin_r = z.sin();

            send_msg.pose.pose.orientation.x = cos_a*cos_b*cos_r - sin_a*sin_b*sin_r;
            send_msg.pose.pose.orientation.y = sin_a*cos_b*cos_r + cos_a*sin_b*sin_r;
            send_msg.pose.pose.orientation.z = cos_a*sin_b*cos_r - sin_a*cos_b*sin_r;
            send_msg.pose.pose.orientation.w = cos_a*cos_b*sin_r + sin_a*sin_b*cos_r;

            let _ = publisher.send(&send_msg);
        })
    );

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    loop {
        selector.wait()?;
    }
}