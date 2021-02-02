#include <turtlebot3_diff_drive_exercise/diff_drive_control.h>

#include <tf/tf.h>



namespace turtlebot3
{
DiffDriveControl::DiffDriveControl(ros::NodeHandle& nh)
  : is_active_(false)
  , first_heading_alignment_done_(false),
    goal_position_reached_(false)
{
  max_linear_vel_ = nh.param("diff_drive/max_linear_vel", 1.0);
  max_linear_acc_ = nh.param("diff_drive/max_linear_acc", 1.0);
  max_angular_vel_ = nh.param("diff_drive/max_angular_vel", 1.0);
  max_angular_acc_ = nh.param("diff_drive/max_angular_acc", 1.0);

  pos_tolerance_ = nh.param("diff_drive/pos_tolerance", 0.01);
  angular_tolerance_ = nh.param("diff_drive/angular_tolerance", 0.01);

  allow_backward_ = nh.param("diff_drive/allow_backward", true);

  p_gain_ = nh.param("diff_drive/p_gain", 1.0);
}

DiffDriveControl::~DiffDriveControl()
{
}

void DiffDriveControl::reset()
{
  is_active_ = false;
  first_heading_alignment_done_ = false;
  goal_position_reached_ = false;
  goal_pose_ = geometry_msgs::Pose();
}

void DiffDriveControl::setGoalPose(const geometry_msgs::Pose& goal)
{
  is_active_ = true;
  goal_pose_ = goal;
}

void DiffDriveControl::stop()
{
  reset();
}

bool DiffDriveControl::isActive() const
{
  return is_active_;
}

geometry_msgs::Twist DiffDriveControl::computeTwist(const ros::Duration& time_diff, const geometry_msgs::Pose& robot_pose, const nav_msgs::Odometry& odom)
{
  geometry_msgs::Twist twist;

  double robot_yaw = tf::getYaw(robot_pose.orientation);
  double goal_yaw = tf::getYaw(goal_pose_.orientation);

  /// | Implement differential drive control here |
  /// v                                           v

  /// ^                                           ^
  /// | ------ Differential drive control end --- |

  if (is_active_)
  {
    return twist;
  }
  else
    return geometry_msgs::Twist(); // stops robot
}

/// | ------ Additional Methods ------ |
/// v                                  v

/// ^                                  ^
/// | ------ Additional Methods end -- |

} // namespace
