#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "joy_missile_controller/joy_missile_controller.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<joy_missile_controller::JoyMissileController>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}
