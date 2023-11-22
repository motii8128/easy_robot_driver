using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace ROS2
{

/// <summary>
/// An example class provided for testing of basic ROS2 communication
/// </summary>
public class UnityController : MonoBehaviour
{
    // Start is called before the first frame update
    private ROS2UnityComponent ros2Unity;
    private ROS2Node ros2Node;
    private IPublisher<geometry_msgs.msg.Twist> unity_controller;
    private ISubscription<nav_msgs.msg.Odometry> sub_odom;

    private nav_msgs.msg.Odometry get_odom;
    double sim_pose_x;
    double sim_pose_y;
    double sim_theta;

    public RectTransform robo_sim;
    public Text linear_meter;
    public Text angular_meter;


    void Start()
    {
        ros2Unity = GetComponent<ROS2UnityComponent>();

        get_odom = new nav_msgs.msg.Odometry();
        sim_pose_x = 0.0;
        sim_pose_y = 0.0;
        sim_theta = 0.0;
    }

    void Update()
    {
        if (ros2Unity.Ok())
        {
            if (ros2Node == null)
            {
                ros2Node = ros2Unity.CreateNode("UnityControllerNode");
                unity_controller = ros2Node.CreatePublisher<geometry_msgs.msg.Twist>("/turtle1/cmd_vel");
                sub_odom = ros2Node.CreateSubscription<nav_msgs.msg.Odometry>("/odom", msg => get_odom = msg);
            }

            sim_pose_x = get_odom.Pose.Pose.Position.X*50;
            sim_pose_y = get_odom.Pose.Pose.Position.Y*50;
            sim_theta = get_odom.Pose.Pose.Position.Z * 60;


            linear_meter.text = (get_odom.Twist.Twist.Linear.X).ToString();
            angular_meter.text =(get_odom.Twist.Twist.Angular.X).ToString();


            robo_sim.position = new Vector3((int)(sim_pose_x), (int)(sim_pose_y), 0);
            robo_sim.rotation = Quaternion.Euler(0, 0, (int)(sim_theta));

            geometry_msgs.msg.Twist msg = new geometry_msgs.msg.Twist();
            msg.Linear.X = Fix(Input.GetAxis("left_x"));
            msg.Linear.Y = Fix(-1.0 * Input.GetAxis("left_y"));
            msg.Angular.Z = Fix(-1.0 * Input.GetAxis("right_x"));

            
            unity_controller.Publish(msg);
        }
    }

    double Fix(double value)
    {
        if(value > 0.0)
        {
            if(value < 0.1)
            {
                value = 0.0;
            }
        }
        else
        {
            if(value > -0.1)
            {
                value = 0.0;
            }
        }

        return value;
    }
}

}  // namespace ROS2
