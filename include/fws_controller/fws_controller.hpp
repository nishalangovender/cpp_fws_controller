#ifndef FWS_CONTROLLER_HPP_
#define FWS_CONTROLLER_HPP_

#include <memory>

#include "fws_controller_parameters.hpp"
#include "steering_controllers_library/steering_controllers_library.hpp"

namespace fws_controller
{

    // State Interfaces
    static constexpr size_t STATE_TRACTION_WHEEL = 0;
    static constexpr size_t STATE_STEER_AXIS = 1;

    // Command Interfaces
    static constexpr size_t CMD_TRACTION_WHEEL = 0;
    static constexpr size_t CMD_STEER_WHEEL = 1;

    static constexpr size_t NR_STATE_ITFS = 2;
    static constexpr size_t NR_CMD_ITFS = 2;
    static constexpr size_t NR_REF_ITFS = 2;

    class FWSController : public steering_controllers_library::SteeringControllersLibrary
    {

    public:

        FWSController();

        controller_interface::CallbackReturn configure_odometry() override;
        bool update_odometry(const rclcpp::Duration & period) override;
        void initialize_implementation_parameter_listener() override;

        // Command and State Interfaces
        // controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        // controller_interface::InterfaceConfiguration state_interface_configuration() const override;
        // controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        // controller_interface::CallbackReturn on_init() override;
        // controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        // controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    protected:

        std::shared_ptr<fws_controller::ParamListener> fws_param_listener_;
        fws_controller::Params fws_params_;

        // Parameters from ROS for fws_controller
        // std::vector<std::string> steering_joint_names_;
        // std::vector<std::string> wheel_joint_names_;

        // // Subscription from Steering Commands
        // rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr steering_command_subscriber_;

        // // Subscription for Wheels Commands (Velocity)
        // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheels_velocity_command_subscriber_;

        // // Realtime Buffer for Wheels Velocity Commands
        // realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> wheels_velocity_command_buffer_;

        // // Realtime Buffer for Steering Position Commands
        // realtime_tools::RealtimeBuffer<std::shared_ptr<trajectory_msgs::msg::JointTrajectory>> steering_position_command_buffer_;

        // // Command Interfaces
        // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> steering_position_command_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> wheels_velocity_command_interface_;

        // // State Interfaces
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> steering_joint_state_interface_;
        // std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> wheels_joint_state_interface_;

        // // Command State Interface Map
        // std::unordered_map<std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *> command_interface_map_ = {
        //     {"steering_position", &steering_position_command_interface_},
        //     {"wheel_velocity", &wheels_velocity_command_interface_}};

        // // Control Logic Steering and Wheels
        // void controlSteering(const std::vector<double> &steering_commands);
        // void controlWheels(double front_wheel_velocity, double rear_wheel_velocity);
    };

} // namespace fws_controller

#endif // FWS_CONTROLLER_HPP_