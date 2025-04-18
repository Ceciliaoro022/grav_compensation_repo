#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

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

class TransformListenerNode: public rclcpp::Node
{
public:
    TransformListenerNode() : Node("transform_listener_node")
    {
        // Initialize the TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rotation_matrix_pub = this->create_publisher<std_msgs::msg::Float64MultiArray >("/rotation_matrix", 10);

        // Timer to continuously check for the transform at a specified rate
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), std::bind(&TransformListenerNode::getTransform, this));
    }

private:
    // Declare shared pointers for TF buffer and listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rotation_matrix_pub;

    void getTransform()
    {
        try
        {
            // Look up the transform between "world" and "mini_base_link"
            geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(
                "world",    // target frame
                "base_link",  // source frame
                rclcpp::Time(0), // specify that we want the latest available transform
                std::chrono::seconds(1)); // Timeout after 1 second if no transform is found

            // Print the transform
            std::cout << "Transform from world to mini_base_link:" << std::endl;
            std::cout << "Translation: [" 
                      << transformStamped.transform.translation.x << ", "
                      << transformStamped.transform.translation.y << ", "
                      << transformStamped.transform.translation.z << "]" << std::endl;

            std::cout << "Rotation: [" 
                      << transformStamped.transform.rotation.x << ", "
                      << transformStamped.transform.rotation.y << ", "
                      << transformStamped.transform.rotation.z << ", "
                      << transformStamped.transform.rotation.w << "]" << std::endl;
        }
        catch (const tf2::TransformException &ex)
        {
            // Handle error if transformation can't be found
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }

        Eigen::Matrix3d rotation_matrix_for_mini;

        std_msgs::msg::Float64MultiArray rotation_matrix_for_mini_msg;
        rotation_matrix_for_mini_msg.data = {
            rotation_matrix_for_mini(0,0), rotation_matrix_for_mini(0,1), rotation_matrix_for_mini(0,2),
            rotation_matrix_for_mini(1,0), rotation_matrix_for_mini(1,1), rotation_matrix_for_mini(1,2),
            rotation_matrix_for_mini(2,0), rotation_matrix_for_mini(2,1), rotation_matrix_for_mini(2,2),
        };

        rotation_matrix_pub->publish(rotation_matrix_for_mini_msg);
    };


};

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create and spin the node
    rclcpp::spin(std::make_shared<TransformListenerNode>());

    // Shut down ROS 2 when done
    rclcpp::shutdown();
    return 0;
}
