#include <turtlebot3_diff_drive_exercise/diff_drive.h>



namespace turtlebot3
{
DiffDrive::DiffDrive(ros::NodeHandle& nh)
{
  // get parameters
  wheel_radius_ = nh.param("diff_drive/wheel_radius", 0.033);
  wheel_seperation_ = nh.param("diff_drive/wheel_seperation", 0.160);
  max_wheel_vel_ = nh.param("diff_drive/max_wheel_vel", 1.0);
}

DiffDrive::~DiffDrive()
{
}

void DiffDrive::computeWheelVelocities(const geometry_msgs::Twist twist_msg, double& wheel_l_vel, double& wheel_r_vel)
{
  /// | Implement wheel velocity computation here |
  /// v                                           v

  /// ^                                                   ^
  /// | ------ Wheel velocity computation end ----------- |
}
} // namespace
