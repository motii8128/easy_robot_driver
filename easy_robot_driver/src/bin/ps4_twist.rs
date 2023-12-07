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

struct JoyPS4
{
    pub joy_left_x:f32,
    pub joy_left_y:f32,
    pub joy_right_x:f32,
    pub joy_right_y:f32,
    pub up_down:f32,
    pub left_right:f32,
    pub circle:f32,
    pub cross:f32,
    pub triangle:f32,
    pub cube:f32,
    pub r1:f32,
    pub r2:f32,
    pub l1:f32,
    pub l2:f32,
}

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

fn convert_struct(msg:sensor_msgs::msg::Joy)->JoyPS4
{
    let result = JoyPS4{
        joy_left_x:*msg.axes.as_slice().get(0).unwrap(),
        joy_left_y:*msg.axes.as_slice().get(1).unwrap(),
        joy_right_x:*msg.axes.as_slice().get(3).unwrap(),
        joy_right_y:*msg.axes.as_slice().get(4).unwrap(),
    };

    result
}

fn fix(value:f32)->f32
{
    let result = 0.0;

    if value < 0.1
    {
        
    }
}