## fws_controller

ROS2 Humble 4WS Controller for ROS2 Control.

NOTE: The information following is from Differential Drive Controller (https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html) and will be updated to this controller.

==========================================================================================================================================================================================================

Controller for mobile robots with 4WS drive.

As input it takes heading and velocity commands for the robot body, which are translated to steering and wheel commands for the 4WS drive base.

Odometry is computed from hardware feedback and published.

## Controller Interfaces

# Feedback

As feedback interface type the joints' position (hardware_interface::HW_IF_POSITION) or velocity (hardware_interface::HW_IF_VELOCITY, if parameter position_feedback=false) are used.

# Output

Joints' position (hardware_interface::HW_IF_POSITION) and velocity (hardware_interface::HW_IF_VELOCITY) are used.

## ROS2 Interfaces

# Subscribers

~/cmd_vel [geomtry_msgs/msg/TwistStamped]
Velocity command for the controller, if use_stamped_vel=true. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

~/cmd_vel_unstamped [geometry_msgs::msg::Twist]
Velocity command for the controller, if use_stamped_vel=false. The controller extracts the x component of the linear velocity and the z component of the angular velocity. Velocities on other components are ignored.

# Publishers

~/odom [nav_msgs::msg::Odometry]
This represents an estimate of the robot’s position and velocity in free space.

/tf [tf2_msgs::msg::TFMessage]
tf tree. Published only if enable_odom_tf=true

~/cmd_vel_out [geometry_msgs/msg/TwistStamped]
Velocity command for the controller, where limits were applied. Published only if publish_limited_velocity=true

# Parameters

linear.x [JointLimits structure]
Joint limits structure for the linear X-axis. The limiter ignores position limits. For details see joint_limits package from ros2_control repository.

angular.z [JointLimits structure]
Joint limits structure for the rotation about Z-axis. The limiter ignores position limits. For details see joint_limits package from ros2_control repository.