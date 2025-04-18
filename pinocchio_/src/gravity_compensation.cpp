#include "pinocchio_/gravity_compensation.h"

GravityCompensationNode::GravityCompensationNode() : Node("gravity_compensation")
{

  this->declare_parameter("urdf_path", "/tmp/kuka_mini_combined.urdf ");
  std::string urdf_path = this->get_parameter("urdf_path").as_string();
  
  pinocchio::urdf::buildModel(urdf_path, model_);
  RCLCPP_INFO(this->get_logger(), "Model loaded: %s", model_.name.c_str());
  
  data_ = pinocchio::Data(model_);
  
  pinocchio::SE3::Vector3 gravity(0, 0, -9.81); //multiply R from the transfrom by the gravity
  model_.gravity.linear(gravity); //Gravity vector assigned to the robot model .. i think is this what it has to change
  
  //joint_name_map 
  joint_name_map_={
    {"mini_joint1", "joint1"},
    {"mini_joint2", "joint2"}
  };


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
  
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model_.nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(model_.nv);
  
  for (const auto& [urdf_joint_name, gazebo_joint_name]:joint_name_map_){
    auto i = std::find(msg->name.begin(), msg->name.end(), gazebo_joint_name); //Looks for joint1 and joint2 from the Gazebo names
    if (i != msg->name.end()){ //If the joint is found..
        size_t gazebo_index = std::distance(msg->name.begin(), i); //Distance between begin to iterator
        pinocchio::JointIndex joint_id = model_.getJointId(urdf_joint_name); //It checks where does pinocchio positioned mini_joint1 from its model

        int q_index = model_.joints[joint_id].idx_q(); //Joint in "gazebo order"
        q(q_index) = msg->position[gazebo_index]; //It orders the joints in "pinocchio order" and passes the angle value
    }
    else { //If joint not found, warning
        RCLCPP_INFO(this->get_logger(), "Joint '%s' not found in /joint_states", gazebo_joint_name.c_str());
    }
  }




  Eigen::VectorXd tau_g = pinocchio::rnea(model_, data_, q, v, a);
  
  auto torque_msg = std::make_unique<std_msgs::msg::Float64MultiArray>(); //unique pointer
  for (const auto& [urdf_joint_name, _]:joint_name_map_){
    pinocchio::JointIndex joint_id = model_.getJointId(urdf_joint_name);
    int v_index = model_.joints[joint_id].idx_v();
    torque_msg->data.push_back(tau_g(v_index));
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