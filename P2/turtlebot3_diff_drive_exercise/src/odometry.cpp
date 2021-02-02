#include <turtlebot3_diff_drive_exercise/odometry.h>

#include <tf/tf.h>



namespace turtlebot3
{
Odometry::Odometry(ros::NodeHandle& nh, size_t rolling_window_size)
{

  odom_msg_.header.frame_id = nh.param("diff_drive/odom_frame", std::string("odom"));
  odom_msg_.child_frame_id = nh.param("diff_drive/base_frame", std::string("base_footprint"));
  wheel_radius_ = nh.param("diff_drive/wheel_radius", 0.033);
  wheel_seperation_ = nh.param("diff_drive/wheel_seperation", 0.160);

  // init Quaternion
  odom_msg_.pose.pose.orientation.w = 1.0;


  // | ----- Initialize new member variables here ----- |
  // v                                                  v

  // ^                                                  ^
  // | ------ Initialize new member variables end ----- |
}

Odometry::~Odometry()
{
}

void Odometry::reset()
{
  odom_msg_.pose = geometry_msgs::PoseWithCovariance();
  odom_msg_.pose.pose.orientation.w = 1.0;
  odom_msg_.twist = geometry_msgs::TwistWithCovariance();
  odom_msg_.header.stamp = ros::Time::now();
}

const nav_msgs::Odometry& Odometry::updateOdometry(const sensor_msgs::JointState::ConstPtr joint_state_msg, const sensor_msgs::Imu::ConstPtr imu_msg)
{
  if (last_joint_state_msg_)
  {
    const sensor_msgs::JointState& joint_state = *joint_state_msg;
    const sensor_msgs::Imu& imu = *imu_msg;

    // compute diff time
    double dt = (joint_state.header.stamp - last_joint_state_msg_->header.stamp).toSec();
    if (dt <= 0.0)
    {
      if (dt < 0.0 && last_joint_state_msg_->header.seq < joint_state.header.seq)
        ROS_WARN("[Odometry] Detected jump back in time in joint states!");
      return odom_msg_;
    }

    // get wheel indeces
    int wheel_l_idx = -1;
    int wheel_r_idx = -1;
    for (size_t i = 0; i < joint_state.name.size(); i++)
    {
      if (joint_state.name[i] == "wheel_left_joint")
        wheel_l_idx = i;
      if (joint_state.name[i] == "wheel_right_joint")
        wheel_r_idx = i;
    }

    if (wheel_l_idx == -1 or wheel_r_idx == -1)
      return odom_msg_;

    /// | Implement odometry computation here |
    /// v                                     v
    ///
    /// Exemplary use of quaternions with TF-Library
    /// tf::Quaternion q;
    /// tf::quaternionMsgToTF(imu.orientation, q);
    /// double yaw = tf::getYaw(q);


    /// ^                                             ^
    /// | ------ Odometry computation end ----------- |


    /// | Store computed values for publication |
    /// v                                       v
    /// Update fields of odometric pose message:
    /// odom_msg_.pose.pose.position
    /// odom_msg_.pose.pose.orientation
    ///
    /// Update fields of odometric instantaneous velocity message:
    /// odom_msg_.twist.twist.linear.x = TODO;
    /// odom_msg_.twist.twist.linear.y = 0.0;
    /// odom_msg_.twist.twist.linear.z = 0.0;
    /// odom_msg_.twist.twist.angular.x = 0.0;
    /// odom_msg_.twist.twist.angular.y = 0.0;
    /// odom_msg_.twist.twist.angular.z = TODO;


    /// ^                                      ^
    /// | ------ Return values end ----------- |

    odom_msg_.header.stamp = joint_state.header.stamp;
  }

  last_joint_state_msg_ = joint_state_msg;

  return odom_msg_;
}
} // namespace
