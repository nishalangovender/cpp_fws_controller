// Nishalan Govender
// Autonomous Parking of a 4WS Vehicle

#include <string>

#include "fws_controller/fws_controller.hpp"

namespace fws_controller
{

  FWSController::FWSController() : position_controllers::JointGroupPositionController()
  {
    interface_name_ = hardware_interface::HW_IF_POSITION;
  }

  // =============================================================================================================================== //

  // Initialization

  // =============================================================================================================================== //

  controller_interface::CallbackReturn FWSController::on_init()
  {

    auto ret = position_controllers::JointGroupPositionController::on_init();
    if (ret != CallbackReturn::SUCCESS)
    {
      return ret;
    }

    try
    {
      get_node()->set_parameter(
          rclcpp::Parameter("interface_name", hardware_interface::HW_IF_POSITION));
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
  }

} // namespace fws_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fws_controller::FWSController, controller_interface::ControllerInterface)