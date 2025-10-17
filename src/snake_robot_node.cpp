#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SnakeRobotNode : public rclcpp::Node
{
public:
  SnakeRobotNode() : Node("snake_robot_node")
  {
    RCLCPP_INFO(this->get_logger(), "Snake robot node started!");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SnakeRobotNode>());
  rclcpp::shutdown();
  return 0;
}

