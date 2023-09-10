#ifndef FWS_CONTROLLER_HPP_
#define FWS_CONTROLLER_HPP_

#include <string>

#include "position_controllers/joint_group_position_controller.hpp"
// #include "velocity_controllers/joint_group_velocity_controller.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"

namespace fws_controller
{
    class FWSController : public position_controllers::JointGroupPositionController
    {
    public:
        FWSController();

        controller_interface::CallbackReturn on_init() override;
    };

} // namespace fws_controller

#endif // FWS_CONTROLLER_HPP_