// Nishalan Govender
// Autonomous Parking of a 4WS Vehicle

#include "fws_controller/fws_controller.hpp"

using config_type = controller_interface::interface_configuration_type;

namespace fws_controller
{

  FWSController::FWSController() : steering_controllers_library::SteeringControllersLibrary() {}

  void FWSController::initialize_implementation_parameter_listener()
  {
    fws_param_listener_ = std::make_shared<fws_controller::ParamListener>(get_node());
  }

  controller_interface::CallbackReturn FWSController::configure_odometry()
  {
    fws_params_ = fws_param_listener_->get_params();

    const double wheelbase = fws_params_.wheelbase;
    const double front_wheel_radius = fws_params_.front_wheel_radius;
    const double rear_wheel_radius = fws_params_.rear_wheel_radius;

    if (params_.front_steering)
    {
      odometry_.set_wheel_params(rear_wheel_radius, wheelbase);
    }
    else
    {
      odometry_.set_wheel_params(front_wheel_radius, wheelbase);
    }

    odometry_.set_odometry_type(steering_odometry::BICYCLE_CONFIG);

    set_interface_numbers(NR_STATE_ITFS, NR_CMD_ITFS, NR_REF_ITFS);

    RCLCPP_INFO(get_node()->get_logger(), "fws odometry configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool FWSController::update_odometry(const rclcpp::Duration &period)
  {
    if (params_.open_loop)
    {
      odometry_.update_open_loop(last_linear_velocity_, last_angular_velocity_, period.seconds());
    }
    else
    {
      const double rear_wheel_value = state_interfaces_[STATE_TRACTION_WHEEL].get_value();
      const double steer_position = state_interfaces_[STATE_STEER_AXIS].get_value();
      if (!std::isnan(rear_wheel_value) && !std::isnan(steer_position))
      {
        if (params_.position_feedback)
        {
          // Estimate linear and angular velocity using joint information
          odometry_.update_from_position(rear_wheel_value, steer_position, period.seconds());
        }
        else
        {
          // Estimate linear and angular velocity using joint information
          odometry_.update_from_velocity(rear_wheel_value, steer_position, period.seconds());
        }
      }
    }
    return true;
  }

} // namespace fws_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fws_controller::FWSController, controller_interface::ChainableControllerInterface)
