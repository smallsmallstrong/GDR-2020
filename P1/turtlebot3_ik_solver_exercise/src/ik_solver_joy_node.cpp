#include <turtlebot3_ik_solver_exercise/ik_solver_joy_node.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>



namespace turtlebot3
{
IkSolverJoyNode::IkSolverJoyNode(ros::NodeHandle& nh)
  : enabled_(false)
{
  // get parameters
  base_frame_ = nh.param("ik/base_frame", std::string("base_link"));
  endeffector_frame_ = nh.param("ik/endeffector_frame", std::string("gripper_link"));

  joint_names_ = nh.param("ik/joints", std::vector<std::string>());

  linear_vel_ = nh.param("max_linear_vel", 0.01);
  angular_vel_ = nh.param("max_angular_vel", 0.1);

  // init subscriber
  cmd_vel_sub_ = nh.subscribe("joy", 1, &IkSolverJoyNode::joyCb, this);

  // init action clients
  joint_traj_ac_.reset(new FollowJointTrajectoryActionClient("joints/arm_trajectory_controller/follow_joint_trajectory", true));
  ik_request_ac_.reset(new IKRequestActionClient("ik_request", true));
}

IkSolverJoyNode::~IkSolverJoyNode()
{
}

void IkSolverJoyNode::joyCb(const sensor_msgs::Joy::ConstPtr joy_msg)
{
  // check if (de-)activation button is pressed
  if (joy_msg->buttons[7])
    enabled_ = true;
  if (joy_msg->buttons[6])
    enabled_ = false;

  if (!enabled_)
    return;

  // get delta x and yaw from joypad (expressed in endeffector frame)
  double dyaw = (joy_msg->buttons[0] - joy_msg->buttons[2]) * angular_vel_;
  tf::Pose dpose_eef(tf::createQuaternionFromYaw(dyaw), tf::Vector3());

  // get endeffector pose
  tf::Stamped<tf::Pose> eef_pose(dpose_eef, ros::Time(0), endeffector_frame_);
  try
  {
    if (tf_listener_.canTransform(base_frame_, endeffector_frame_, ros::Time(0)))
      tf_listener_.transformPose(base_frame_, eef_pose, eef_pose);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[IkSolverJoyNode] %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  // get delta x,y,z from joypad (expressed in base frame)
  double eef_yaw = atan2(eef_pose.getOrigin().getY(), eef_pose.getOrigin().getX());
  double dx = joy_msg->axes[4] * linear_vel_ * cos(eef_yaw);
  double dy = joy_msg->axes[4] * linear_vel_ * sin(eef_yaw);
  double dz = joy_msg->axes[7] * linear_vel_;

  // get delta z and yaw from joypad (expressed in base frame)
  dyaw = std::min(1.0, std::max(-1.0, (double) (joy_msg->axes[3] + joy_msg->axes[6]))) * angular_vel_;
  tf::Pose d_base(tf::createQuaternionFromYaw(dyaw), tf::Vector3(dx, dy, dz));

  // add delta from joystick input
  tf::Pose p = d_base * eef_pose;
  eef_pose.setData(p);

  // prepare request
  turtlebot3_exercise_msgs::IKRequestGoal goal;
  tf::poseStampedTFToMsg(eef_pose, goal.request.goal_pose);

  // send request
  if (ik_request_ac_->sendGoalAndWait(goal, ros::Duration(1.0)) == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    turtlebot3_exercise_msgs::IKRequestResultConstPtr result = ik_request_ac_->getResult();

    // only execute trajectory, when left trigger is pressed
    bool execute_ik = joy_msg->axes[2] < 0.0;
    if (execute_ik)
      moveArm(result->result);

    ROS_INFO("SUCCESS!");
  }
  else
    ROS_ERROR("[IkSolverJoyNode] Didn't get any IK result within 1s from IK solver.");
}

void IkSolverJoyNode::moveArm(const turtlebot3_exercise_msgs::IKResult& ik_result)
{
  // check connection to action server
  if (!joint_traj_ac_->waitForServer(ros::Duration(1.0)))
  {
    ROS_ERROR("[IkSolverJoyNode] Can't connect to joint trajectory server!");
    return;
  }

  // transform goal joint angles into a joint trajectory message
  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;

  trajectory.joint_names = joint_names_;

  trajectory_msgs::JointTrajectoryPoint point;

  for (const std::string& joint_name : joint_names_)
  {
    bool found = false;

    for (size_t i = 0; i < ik_result.name.size() && !found; i++)
    {
      if (ik_result.name[i] == joint_name)
      {
        point.positions.push_back(ik_result.position[i]);
        found = true;
      }
    }

    if (!found)
    {
      ROS_WARN("[IkSolverJoyNode] Missing joint '%s' in IK solution!", joint_name.c_str());
      return;
    }
  }

  point.time_from_start = ros::Duration(0.25);

  trajectory.points.push_back(point);

  // send action goal
  joint_traj_ac_->sendGoal(goal);
}
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_ik_solver_joy");

  ros::NodeHandle nh;
  turtlebot3::IkSolverJoyNode node(nh);
  ros::spin();

  return 0;
}
