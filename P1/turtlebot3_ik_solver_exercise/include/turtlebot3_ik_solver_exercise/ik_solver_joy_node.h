#ifndef TURTLEBOT3_IK_SOVLER_JOY_NODE_H__
#define TURTLEBOT3_IK_SOVLER_JOY_NODE_H__

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>

#include <sensor_msgs/Joy.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <turtlebot3_exercise_msgs/IKRequestAction.h>



namespace turtlebot3
{
class IkSolverJoyNode
{
public:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionClient;
  typedef actionlib::SimpleActionClient<turtlebot3_exercise_msgs::IKRequestAction> IKRequestActionClient;

  IkSolverJoyNode(ros::NodeHandle& nh);
  virtual ~IkSolverJoyNode();

protected:
  void joyCb(const sensor_msgs::Joy::ConstPtr joy_msg);

  // helper
  void moveArm(const turtlebot3_exercise_msgs::IKResult& ik_result);

  // class members
  tf::TransformListener tf_listener_;

  // parameters
  std::string base_frame_;
  std::string endeffector_frame_;

  std::vector<std::string> joint_names_;

  double linear_vel_;
  double angular_vel_;

  bool enabled_;

  // subscriber
  ros::Subscriber cmd_vel_sub_;

  // action clients
  boost::shared_ptr<FollowJointTrajectoryActionClient> joint_traj_ac_;
  boost::shared_ptr<IKRequestActionClient> ik_request_ac_;
};
}

#endif
