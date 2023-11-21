use safe_drive::{
    context::Context,
    error::DynError,
    msg::common_interfaces::{geometry_msgs, std_msgs},
    logger::Logger,
    pr_info
};

use ros2_rust_util::get_f64_parameter;

fn main()->Result<(), DynError>
{
    let ctx = Context::new()?;

    let node = ctx.create_node("simple_mechanum_driver", None, Default::default())?;

    let subscriber = node.create_subscriber::<geometry_msgs::msg::Twist>("/cmd_vel", None)?;

    let publisher_fl = node.create_publisher::<std_msgs::msg::Float32>("/wheel/front_left", None)?;
    let publisher_fr = node.create_publisher::<std_msgs::msg::Float32>("/wheel/front_right", None)?;
    let publisher_rl = node.create_publisher::<std_msgs::msg::Float32>("/wheel/raw_left", None)?;
    let publisher_rr = node.create_publisher::<std_msgs::msg::Float32>("/wheel/raw_right", None)?;

    let mut selector = ctx.create_selector()?;

    let move_speed_rate = get_f64_parameter(node.get_name(), "move_speed_rate", 1.0) as f32;
    let rotation_speed_rate = get_f64_parameter(node.get_name(), "rotation_speed_rate", 1.0) as f32;

    let log = Logger::new(node.get_name());
    pr_info!(log, "Start {}", node.get_name());

    selector.add_subscriber(
        subscriber,
        Box::new(move |msg| {
            let vec_x = msg.linear.x as f32;
            let vec_y = msg.linear.y as f32;
            let rotation_pow = msg.angular.z as f32;

            let mut fl_msg = std_msgs::msg::Float32::new().unwrap();
            let mut fr_msg = std_msgs::msg::Float32::new().unwrap();
            let mut rl_msg = std_msgs::msg::Float32::new().unwrap();
            let mut rr_msg = std_msgs::msg::Float32::new().unwrap();

            let calc_x = move_speed_rate * (f32::sqrt(2.0) / 2.0) * vec_x;
            let calc_y = move_speed_rate * (f32::sqrt(2.0) / 2.0) * vec_y;
            let calc_rota = rotation_speed_rate * rotation_pow;

            fl_msg.data = calc_x - calc_y + calc_rota;
            fr_msg.data = calc_x - calc_y - calc_rota; 
            rl_msg.data = -calc_x - calc_y + calc_rota;
            rr_msg.data = -calc_x - calc_y - calc_rota;

            let _ = publisher_fl.send(&fl_msg);
            let _ = publisher_fr.send(&fr_msg);
            let _ = publisher_rl.send(&rl_msg);
            let _ = publisher_rr.send(&rr_msg);
        }),
    );

    loop {
        selector.wait()?;
    }

}