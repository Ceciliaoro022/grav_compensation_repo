#include "pinocchio_/gravity_compensation.h"

#include "pinocchio_/rrbot_description.hpp"


using namespace pinocchio;


GravityCompensationNode::GravityCompensationNode() : Node("gravity_compensation")
{

  model_ = buildMini2RModel();
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81);
  model_.gravity.linear(gravity); //Gravity vector assigned to the robot model
  
  // joint state subscriber
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, 
    std::bind(&GravityCompensationNode::joint_state_callback, this, std::placeholders::_1));
  
  //gravity torque publisher
  gravity_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/gravity_torques", 10);
    
  RCLCPP_INFO(this->get_logger(), "Gravity compensation node started");
}

void GravityCompensationNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  
  //New
  std::vector<std::string> target_joint_names = {"joint1", "joint2"};  
  Eigen::VectorXd q(model_.nv);

  for (size_t i = 0; i < target_joint_names.size(); ++i) {
    auto it = std::find(msg->name.begin(), msg->name.end(), target_joint_names[i]); //Looks for joint1 and 2 from the target names above
    if (it != msg->name.end()) { //If the joint was found..
      size_t index = std::distance(msg->name.begin(), it); //Distance between begin to iterator, so index of the found joint
      q(i) = msg->position[index]; //Checks the q from the actual position where my joint is in the /joint_states
    } else {
      RCLCPP_ERROR(this->get_logger(), "Joint %s not found in /joint_states!", target_joint_names[i].c_str());
      return; // exit to avoid sending bad data
    }
  }

  
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
  
  Eigen::VectorXd tau_g = pinocchio::rnea(model_, data_, q, v, a);
  
  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>(); //unique pointer
  torque_msg->data.resize(tau_g.size());
  
  for (int i = 0; i < tau_g.size(); ++i) {
    torque_msg->data[i] = tau_g(i);
  }
  
  gravity_torque_pub_->publish(std::move(torque_msg));
  
  
}

std::string GravityCompensationNode::eigen_vector_to_string(const Eigen::VectorXd& vec) {
  std::ostringstream oss; //blank to later write in it
  for (int i = 0; i < vec.size(); ++i) {
    oss << vec(i); //takes the value at index i of the vector vec and inserts it into the string stream oss
    if (i < vec.size() - 1) oss << ", "; //checks if the current element is not the last one in the vector
  }
  return oss.str();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GravityCompensationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}