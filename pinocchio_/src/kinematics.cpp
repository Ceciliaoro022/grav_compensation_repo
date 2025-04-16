#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp> //!!!
#include <pinocchio/multibody/fcl.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/serialization/serializable.hpp>

#include <memory>
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <algorithm> 

class ForwardKinematics: public rclcpp::Node
{
public:
    ForwardKinematics()
    : Node("forward_kinematics_node")
    {
        std::string urdf_path = "/tmp/kuka_mini_combined.urdf";

        //Load the urdf model
        pinocchio::urdf::buildModel(urdf_path, model); //model defined in Private
        
        // Create data required by the algorithms
        data = pinocchio::Data(model); 

        std::cout << "model name: " << model.name << std::endl;
        std::cout << "URDF model " << model.name << " has " << model.njoints << " joints" << std::endl;
        std::cout << "It has " << model.nq << " degrees of freedom" << std::endl;

        for (int i = 0; i < model.njoints; i++)
        {
            std::cout << "Joint " << i << " has name " << model.names[i] << std::endl;
        }

        q = pinocchio::neutral(model); //neutral bc I cannot use randomConfiguration bc I use continuous joints and it gives me some issues, but I
        v = Eigen::VectorXd::Zero(model.nv); // velocity vector

        RCLCPP_INFO(this->get_logger(), "Loaded model: %s with DOF: %zu", model.name.c_str(), model.nq); //c_str to pass a string to functions that expect a const char*
        
        Eigen::VectorXd q; //Because Pinocchio uses EigenValues I need to convert them

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ForwardKinematics::jointStateCallback, this, std::placeholders::_1));
        
        forward_kinematics_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_kinematics_info", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ForwardKinematics::computeAndPublishForwarKinematics, this));
    }

private: //For the class
    pinocchio::Model model;
    pinocchio::Data data;      // Added member for data
    Eigen::VectorXd q;         // Added member for q
    Eigen::VectorXd v;
    // If needed, you can also add: Eigen::VectorXd v;

    //They hold the shared pointers to the publisher, subscriber and timer
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_kinematics_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) //If msg from position is empty, return nothing
            return;

        for (size_t i = 0; i < model.names.size(); ++i ) // Changed mode.size() to model.names.size()
        {
            RCLCPP_INFO(this->get_logger(), "Model Joint %zu: %s", i, model.names[i].c_str()); // Changed model.name[i] to model.names[i]
            
            const std::string& joint_name = model.names[i]; //Obtain the joint name from the model from teh current state

            if (joint_name == "universe") continue; //We do not care about universe joint, it is not that I have this joint but
            
            //Finding the joint name in the message
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_name);
            if (it != msg->name.end()) //Check if joint was found
            {
                size_t index = std::distance(msg->name.begin(), it); //calculates the index of the joint in the msg->name vector
                // Ensure we're not out of bounds
                if (index < msg->position.size() && i < q.size())
                {
                    q[i] = msg->position[index]; //if both conditions are satisfied, it assigns the joint’s position from the message to
                }
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Joint %s not found in received joint states.", joint_name.c_str());
            }
        }
    }

    void computeAndPublishForwarKinematics()
    {
        pinocchio::forwardKinematics(model, data, q, v); //I cannot store it inside a variable so let's use oMi
        
        // Typically, data.oMi is a vector of SE3 objects (one for each joint).
        // End-effector is the last joint (index: model.njoints - 1).
        const auto & se3_end_e = data.oMi[model.njoints - 1];

        Eigen::Matrix3d rotation_matrix = se3_end_e.rotation();
        Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(0, 1, 2);

        std_msgs::msg::Float64MultiArray end_e_position_msg;
        end_e_position_msg.data.clear(); //Clear any previous content 
        end_e_position_msg.data.push_back(euler_angles[0]); // Using euler_angles instead of undefined end_e_position
        end_e_position_msg.data.push_back(euler_angles[1]);
        end_e_position_msg.data.push_back(euler_angles[2]);

        forward_kinematics_pub->publish(end_e_position_msg);

        for (size_t i = 0; i < model.njoints; ++i)
        {
            const auto & se3 = data.oMi[i];
            Eigen::AngleAxisd angle_axis(se3.rotation());
            double joint_angle = angle_axis.angle();  // This gives the magnitude of rotation
            std::cout << "Joint " << i << " rotation angle (radians): " << joint_angle << std::endl;
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}