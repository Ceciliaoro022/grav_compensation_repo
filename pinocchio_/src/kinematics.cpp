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

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>



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

        //joint_name_map 
        joint_name_map_={
            {"joint_a1", "joint_a1"},
            {"joint_a2", "joint_a2"},
            {"joint_a3", "joint_a3"},
            {"joint_a4", "joint_a4"},
            {"joint_a5", "joint_a5"},
            {"joint_a6", "joint_a6"},
            {"joint_a7", "joint_a7"},
            {"mini_joint1", "joint1"},
            {"mini_joint2", "joint2"}
        };

        // link_name_map_={
        //     {"mini_base_link", "base_link"}
        // };

       

        std::cout << "model name: " << model.name << std::endl;
        std::cout << "URDF model " << model.name << " has " << model.njoints << " joints" << std::endl;
        std::cout << "It has " << model.nq << " degrees of freedom" << std::endl;

        for (int i = 0; i < model.njoints; i++)
        {
            std::cout << "Joint " << i << " has name " << model.names[i] << std::endl;
        }

        q = pinocchio::neutral(model); //neutral bc I cannot use randomConfiguration bc I use continuous joints and it gives me some issues, but I am using revolute now, so?? check if important **
        v = Eigen::VectorXd::Zero(model.nv); // velocity vector

        RCLCPP_INFO(this->get_logger(), "Loaded model: %s with DOF: %zu", model.name.c_str(), model.nq); //c_str to pass a string to functions that expect a const char*
        
        //Eigen::VectorXd q; //Because Pinocchio uses EigenValues I need to convert them

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&ForwardKinematics::jointStateCallback, this, std::placeholders::_1));
        
        forward_kinematics_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_kinematics_info", 10);

        //mini_base_frame_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/mini_base_frame", 10);

        //rotation_matrix_mini_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rotation_matrix_mini", 10);


        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&ForwardKinematics::computeAndPublishForwarKinematics, this));
    }

private: //For the class
    pinocchio::Model model;
    pinocchio::Data data;      // Added member for data
    Eigen::VectorXd q;         // Added member for q
    Eigen::VectorXd v;
    // If needed, you can also add: Eigen::VectorXd v;

    std::unordered_map<std::string, std::string> joint_name_map_;
    //std::unordered_map<std::string, std::string> link_name_map_;

    //They hold the shared pointers to the publisher, subscriber and timer
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_; 
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr forward_kinematics_pub;
    //rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr mini_base_frame_pub;
    //rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rotation_matrix_mini_pub_;
    
    rclcpp::TimerBase::SharedPtr timer_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) //If msg from position is empty, return nothing
            return;

        for (size_t i = 0; i < model.names.size(); ++i ) // Changed mode.size() to model.names.size()
        {
            
            const std::string& joint_name = model.names[i]; //Obtain the joint name from the model from teh current state

            if (joint_name == "universe") continue; //We do not care about universe joint, it is not that I have this joint but
            

            //Finding the joint name in the message
            for (const auto& [urdf_joint_name, gazebo_joint_name]:joint_name_map_){
                auto i = std::find(msg->name.begin(), msg->name.end(), gazebo_joint_name); //Looks for joint1 and joint2 from the Gazebo names
                if (i != msg->name.end()){ //If the joint is found..
                    size_t gazebo_index = std::distance(msg->name.begin(), i); //Distance between begin to iterator
                    pinocchio::JointIndex joint_id = model.getJointId(urdf_joint_name); //It checks where does pinocchio positioned mini_joint1 from its model
                    if (joint_id == 0) continue; // ID 0 is usually the "universe" joint
                    
                    int q_index = model.joints[joint_id].idx_q(); //Joint in "gazebo order"
                    q(q_index) = msg->position[gazebo_index]; //It orders the joints in "pinocchio order" and passes the angle value

                    //RCLCPP_INFO(this->get_logger(), "Updating joint '%s' from '%s' with value %.4f",
                    //urdf_joint_name.c_str(), gazebo_joint_name.c_str(), msg->position[gazebo_index]);
                }
                else { //If joint not found, warning
                    RCLCPP_INFO(this->get_logger(), "Joint '%s' not found in /joint_states", gazebo_joint_name.c_str());
                }  
            }

        }

        std::cout << "[DEBUG] JointState names: ";
        for (const auto &name : msg->name) std::cout << name << " ";
        std::cout << std::endl;
        std::cout << "[DEBUG] q: " << q.transpose() << std::endl;



    }



    void computeAndPublishForwarKinematics()
    {
        pinocchio::forwardKinematics(model, data, q, v); 

        //To obtain my mini_base_link frame with respect to my world 
        pinocchio::updateGlobalPlacements(model, data); // Updates the position of each frame contained in the model. It gave me an error when using updateFramePlacements (so it depends on the Pinocchio's version)

        std::cout << "q: " << q.transpose() << std::endl;

        //std::string gazebo_link_name = link_name_map_["mini_base_link"];
        pinocchio::FrameIndex mini_base_index = model.getFrameId("mini_link2"); //mini_base_link

        // if (mini_base_index == 0) {
        //     RCLCPP_ERROR(this->get_logger(), "Frame 'mini_base_link' not found in the Pinocchio model!");
        //     return;
        // }

        // for (size_t i = 0; i < model.frames.size(); ++i) {
        //     std::cout << "Frame " << i << " name: " << model.frames[i].name << std::endl;
        // }


        //pinocchio::SE3 world_to_mini = data.oMf[mini_base_index];
        //Eigen::Matrix3d rotation_matrix_mini = world_to_mini.rotation();
        //Eigen::Vector3d rpy = rotation_matrix_mini.eulerAngles(0,1,2);

        pinocchio::SE3 world_to_mini = data.oMf[mini_base_index];
        std::cout << "[DEBUG] Translation: " << world_to_mini.translation().transpose() << std::endl;
        std::cout << "[DEBUG] Rotation:\n" << world_to_mini.rotation() << std::endl;

        pinocchio::JointIndex j1_id = model.getJointId("mini_joint1");
        pinocchio::JointIndex j2_id = model.getJointId("mini_joint2");
        std::cout << "[DEBUG] mini_joint1 q: " << q[model.joints[j1_id].idx_q()] << std::endl;
        std::cout << "[DEBUG] mini_joint2 q: " << q[model.joints[j2_id].idx_q()] << std::endl;


        //Eigen::Vector3d position = world_to_mini.translation();

        //std::cout << "Relative transform between world and mini_base_link:" << std::endl;
        //std::cout << "Translation: " << position.transpose() << std::endl;
        //std::cout << "Euler angles (r,p,y): " << rpy.transpose() << std::endl;  
        //std::cout << "Rotation matrix:\n" << rotation_matrix_mini << std::endl;
        std::cout << "Rotation matrix from world to the mini base_joint:\n " << world_to_mini <<std::endl;


        /*
        std_msgs::msg::Float64MultiArray mini_base_pose_msg;
        mini_base_pose_msg.data.clear();
        mini_base_pose_msg.data.push_back(position[0]);
        mini_base_pose_msg.data.push_back(position[1]);
        mini_base_pose_msg.data.push_back(position[2]);
        mini_base_pose_msg.data = {rpy[0], rpy[1], rpy[2]};

        mini_base_frame_pub->publish(mini_base_pose_msg);
        */

        //------------------------------------------------------------------------------------------
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

        /*
        for (size_t i = 0; i < model.njoints; ++i)
        {
            const auto & se3 = data.oMi[i];
            Eigen::AngleAxisd angle_axis(se3.rotation());
            double joint_angle = angle_axis.angle();  // This gives the magnitude of rotation
            std::cout << "Joint " << i << " rotation angle (radians): " << joint_angle << std::endl;
        }

        std_msgs::msg::Float64MultiArray rot_msg;
        rot_msg.data.resize(9);  // 3x3 matrix

        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                rot_msg.data[i * 3 + j] = rotation_matrix_mini(i, j);  // row-major

        rotation_matrix_mini_pub_->publish(rot_msg);
        */

        

    }


    

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardKinematics>());
    rclcpp::shutdown();
    return 0;
}