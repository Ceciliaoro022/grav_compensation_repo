#ifndef GRAVITY_COMPENSATION_H
#define GRAVITY_COMPENSATION_H

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <sstream>
#include <unordered_map>

class GravityCompensationNode : public rclcpp::Node
{
public:
  GravityCompensationNode();

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void rotation_matrix_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  std::string eigen_vector_to_string(const Eigen::VectorXd& vec);

  pinocchio::Model model_;
  pinocchio::Data data_;
  std::unordered_map<std::string, std::string> joint_name_map_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr gravity_torque_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotation_matrix_sub;
};

#endif 