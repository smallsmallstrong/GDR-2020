#ifndef TURTLEBOT3_IK_NEWTON_NODE_H__
#define TURTLEBOT3_IK_NEWTON_NODE_H__

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <turtlebot3_exercise_msgs/FKRequestAction.h>
#include <turtlebot3_exercise_msgs/IKRequestAction.h>

#include <turtlebot3_ik_solver_exercise/ik_marker.h>
#include <turtlebot3_ik_solver_exercise/ik_solver.h>



namespace turtlebot3
{
class IKSolverNode
{
public:
  typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionClient;
  typedef actionlib::SimpleActionServer<turtlebot3_exercise_msgs::FKRequestAction> FKRequestActionServer;
  typedef actionlib::SimpleActionServer<turtlebot3_exercise_msgs::IKRequestAction> IKRequestActionServer;

  IKSolverNode(ros::NodeHandle& nh);
  virtual ~IKSolverNode();

  void update();

protected:
  // UI callbacks
  void setOnlineIKEnabled(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void setReuseIKSolution(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void solveIK(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void snapToCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);
  void executeIk(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback);

  // IK Solver callbacks
  void fkResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr result);
  void ikResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr result);

  // ROS API callbacks
  void fkRequestActionGoalCb();
  void fkRequestActionResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr result);
  void ikRequestActionGoalCb();
  void ikRequestActionResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr result);

  void jointStateCb(const sensor_msgs::JointState::ConstPtr msg);

  // class members
  IKSolver ik_solver_;

  IKMarker::Ptr ik_marker_;

  tf::TransformListener tf_listener_;

  // parameters
  std::string base_frame_;
  std::string endeffector_frame_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> fixed_joint_names_;
  std::vector<std::string> link_names_;

  bool online_ik_enabled_;
  bool reuse_ik_solution_enabled_;
  bool has_ik_result_;
  bool auto_snap_to_current_pose_;

  double endeffector_speed_;
  double speed_scale_;

  sensor_msgs::JointState::ConstPtr last_joint_state_msg_;

  interactive_markers::MenuHandler::EntryHandle ik_request_handle_;
  interactive_markers::MenuHandler::EntryHandle execute_handle_;

  // subscriber
  ros::Subscriber joint_state_sub_;

  // publisher
  ros::Publisher robot_state_pub_;

  // action client
  boost::shared_ptr<FollowJointTrajectoryActionClient> joint_traj_ac_;

  // action server
  boost::shared_ptr<FKRequestActionServer> fk_request_as_;
  boost::shared_ptr<IKRequestActionServer> ik_request_as_;
};
}

#endif
