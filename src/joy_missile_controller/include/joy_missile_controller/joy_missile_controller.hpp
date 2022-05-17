#ifndef JOY_MISSILE_CONTROLLER_H
#define JOY_MISSILE_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include "joy_missile_controller/joy_missile_controller_export.h"

namespace joy_missile_controller
{

class JOY_MISSILE_CONTROLLER_EXPORT JoyMissileController : public rclcpp::Node
{
public:
  explicit JoyMissileController(const rclcpp::NodeOptions& options);

  virtual ~JoyMissileController();

private:
  struct Impl;
  Impl* pimpl_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle;  
};

}  // namespace teleop_twist_joy

#endif  // JOY_MISSILE_CONTROLLER_H