use safe_drive::{
    context::Context,
    error::DynError,
    logger::Logger,
    pr_info,
    msg::{common_interfaces::{sensor_msgs, geometry_msgs}, F32Seq, I32Seq},
};

use easy_robot_driver::{Axes, Buttons, JoyPS4};

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

            let joy_ps4 = JoyPS4{
                axes:get_axis(&msg.axes),
                buttons:get_button(&msg.buttons)
            };

            send_msg.linear.x = fix(joy_ps4.axes.joy_left_x) as f64;
            send_msg.linear.y = fix(joy_ps4.axes.joy_left_y) as f64;
            send_msg.angular.z = fix(joy_ps4.axes.joy_right_x) as f64;

            let _ = publisher.send(&send_msg);
        })
    );

    let log = Logger::new(node.get_name());

    pr_info!(log, "Start {}", node.get_name());

    loop {
        selector.wait()?;
    }
}



fn get_axis(axes:&F32Seq<0>)->Axes
{
    let result = Axes {
        // axes
        joy_left_x:*axes.as_slice().get(0).unwrap(),
        joy_left_y:*axes.as_slice().get(1).unwrap(),
        l2_axes:*axes.as_slice().get(2).unwrap(),
        joy_right_x:*axes.as_slice().get(3).unwrap(),
        joy_right_y:*axes.as_slice().get(4).unwrap(),
        r2_axes:*axes.as_slice().get(5).unwrap(),
        left_right:*axes.as_slice().get(6).unwrap(),
        up_down:*axes.as_slice().get(7).unwrap(),
    };

    result
}

fn get_button(button:&I32Seq<0>)->Buttons
{
    let result = Buttons {
        // buttons
        circle:*button.as_slice().get(1).unwrap(),
        cross:*button.as_slice().get(0).unwrap(),
        triangle:*button.as_slice().get(2).unwrap(),
        cube:*button.as_slice().get(3).unwrap(),
        r1:*button.as_slice().get(5).unwrap(),
        l1:*button.as_slice().get(4).unwrap(),
        l2_button:*button.as_slice().get(6).unwrap(),
        r2_button:*button.as_slice().get(7).unwrap(),
    };

    result
}

fn fix(value:f32)->f32
{
    let mut result = 0.0;

    if value > 0.1
    {
        result = value;
    }

    result
}