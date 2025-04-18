// this node has its class fully defined in the cpp file
// for larger nodes, the class can be classically split in header / source


// include any thing required - do not forget to use the .hpp extension for ROS 2 files
#include <rclcpp/rclcpp.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>
#include <algorithm>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ik_client.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>


using namespace std::chrono_literals;
using baxter_core_msgs::msg::JointCommand;
using baxter_core_msgs::srv::SolvePositionIK;

namespace lab3_puppet
{

class PuppetNode : public rclcpp::Node
{
public:
  PuppetNode(rclcpp::NodeOptions options) : Node("puppet", options)
  {
    // init whatever is needed for your node

    // init command message for left arm

    // init publisher to left arm command
    publisher = create_publisher<JointCommand>("/robot/limb/left/joint_command", 10);   // topic + QoS

    // init timer - the function publishCommand() should called with the given rate
    timer = this->create_wall_timer(5ms, std::bind(&PuppetNode::publishCommand, this));

    // IK service wrapper into IKNode
    ik_node.init("ik_node","/ExternalTools/left/PositionKinematicsNode/IKService");
  }
  
private:

  // declare member variables for command publisher and timer
  rclcpp::Publisher<JointCommand>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  ServiceNodeSync<SolvePositionIK> ik_node;

  // TF 2 stuff
  tf2_ros::Buffer tf_buffer{get_clock()};                // stores all previous elementary transforms in a tree
  tf2_ros::TransformListener tf_listener{tf_buffer};   // subscribes to /tf

  void publishCommand()
  {
    // check if the transform from base to left_gripper_desired is available
    if(tf_buffer.canTransform("left_gripper_desired", "base", tf2::TimePointZero, tf2::durationFromSec(1.0)))
    {
      // get this transform with tf_buffer.lookupTransform("base", "left_gripper_desired", ...
       geometry_msgs::msg::TransformStamped desired_transform = tf_buffer.lookupTransform("base","left_gripper_desired",tf2::TimePointZero, tf2::durationFromSec(1.0));


      // build service request SolvePositionIK::Request from obtained transform
      SolvePositionIK::Request req;

      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = desired_transform.header;                         //get time stamp

      pose_stamped.pose.position.x = desired_transform.transform.translation.x;     //get the position from the transform
      pose_stamped.pose.position.y = desired_transform.transform.translation.y;
      pose_stamped.pose.position.z = desired_transform.transform.translation.z;

      pose_stamped.pose.orientation.x = desired_transform.transform.rotation.x;
      pose_stamped.pose.orientation.y = desired_transform.transform.rotation.y;
      pose_stamped.pose.orientation.z = desired_transform.transform.rotation.z;
      pose_stamped.pose.orientation.w = desired_transform.transform.rotation.w;

      req.pose_stamp.push_back(pose_stamped);



      // call service and get response
      if(SolvePositionIK::Response res; ik_node.call(req, res))
      {
//          std::cout << "Called service" << std::endl;

          // call to IK was successfull, check if the solution is valid
          if(res.result_type[0] >= 0){                                                 //because if it is -1 or -2 failed or in collision, all other seed values are for valid positions. (https://rethinkrobotics.github.io/intera_sdk_docs/5.0.4/intera_core_msgs/html/srv/SolvePositionIK.html)
//              std::cout << "Got valid result" << std::endl;

              // copy response data to joint command and publish to left arm
              baxter_core_msgs::msg::JointCommand command_msg;
              command_msg.mode = 1;

              sensor_msgs::msg::JointState  joint = res.joints[0];    //for each joint i can get many IK solutions for different positions, here we just use 1 (index[0])

              for (int i=0; i<joint.name.size(); ++i){                //Here i loop through the different joints in the robot
//                  std::cout << "building message" << std::endl;
                  command_msg.names.push_back(joint.name[i]);
                  command_msg.command.push_back(joint.position[i]);
              }

              publisher->publish(command_msg);
//              std::cout << "Published" << std::endl;
          }

      }
    }
  }
};
}


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab3_puppet::PuppetNode)

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lab3_puppet::PuppetNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}