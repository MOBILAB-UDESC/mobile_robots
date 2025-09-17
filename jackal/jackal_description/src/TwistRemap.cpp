/**
 * @file TwistRemap.cpp
 * @brief ROS2 node to convert Twist messages to TwistStamped messages.
 *
 * This node subscribes to velocity commands of type geometry_msgs::msg::Twist
 * (commonly published on the /cmd_vel topic by ROS2 navigation stacks)
 * and republishes them as geometry_msgs::msg::TwistStamped messages.
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistToTwistStamped : public rclcpp::Node
{
public:
  TwistToTwistStamped(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("twist_to_twist_stamped", options)
  {
    twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TwistToTwistStamped::cmd_vel_callback, this, std::placeholders::_1));

    twist_stamped_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "diff_drive_base_controller/cmd_vel", 10);
  }

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {

    if (!this->has_parameter("frame_id")) {
      this->declare_parameter("frame_id", "base_link");
    }

    auto twist_stamped_msg = std::make_shared<geometry_msgs::msg::TwistStamped>();

    // Add timestamp and frame_id to the header
    twist_stamped_msg->header.stamp = this->now();
    twist_stamped_msg->header.frame_id = this->get_parameter("frame_id").as_string();

    // Copy velocity data from the twist message
    twist_stamped_msg->twist = *msg;

    // Publish the stamped version
    twist_stamped_publisher_->publish(*twist_stamped_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<TwistToTwistStamped>(options);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}