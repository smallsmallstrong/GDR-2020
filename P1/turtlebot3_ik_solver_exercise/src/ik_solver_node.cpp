#include <turtlebot3_ik_solver_exercise/ik_solver_node.h>



namespace turtlebot3
{
IKSolverNode::IKSolverNode(ros::NodeHandle& nh)
  : ik_solver_(nh)
  , has_ik_result_(false)
  , auto_snap_to_current_pose_(true)
{
  // get parameters
  base_frame_ = nh.param("ik/base_frame", std::string("base_link"));
  endeffector_frame_ = nh.param("ik/endeffector_frame", std::string("gripper_link"));

  joint_names_ = nh.param("ik/joints", std::vector<std::string>());
  fixed_joint_names_ = nh.param("ik/fixed_joints", std::vector<std::string>());
  link_names_ = nh.param("ik/links", std::vector<std::string>());

  online_ik_enabled_ = nh.param("ik/online_ik", true);
  reuse_ik_solution_enabled_ = nh.param("ik/reuse_ik_solution", true);

  endeffector_speed_ = nh.param("ik/endeffector_speed", 0.1);
  speed_scale_ = nh.param("ik/speed_scale", 1.0);

  // init marker
  ik_marker_.reset(new turtlebot3::IKMarker("ik_solver", base_frame_, nh.param("ik/marker_scale", 1.0)));

  // init marker menu
  ik_marker_->insertCheckableMenuItem("Online IK", online_ik_enabled_ ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED, boost::bind(&IKSolverNode::setOnlineIKEnabled, this, _1));
  ik_marker_->insertCheckableMenuItem("Reuse IK Solution", reuse_ik_solution_enabled_ ? interactive_markers::MenuHandler::CHECKED : interactive_markers::MenuHandler::UNCHECKED, boost::bind(&IKSolverNode::setReuseIKSolution, this, _1));
  ik_request_handle_ = ik_marker_->insertMenuItem("Solve IK", boost::bind(&IKSolverNode::solveIK, this, _1));
  ik_marker_->setVisible(ik_request_handle_, !online_ik_enabled_);
  ik_marker_->insertMenuItem("Snap to Arm", boost::bind(&IKSolverNode::snapToCurrentPose, this, _1));
  execute_handle_ = ik_marker_->insertMenuItem("Execute", boost::bind(&IKSolverNode::executeIk, this, _1));
  ik_marker_->setVisible(execute_handle_, false);

  // subscriber
  joint_state_sub_ = nh.subscribe("joints/joint_states", 1, &IKSolverNode::jointStateCb, this);

  // publisher
  robot_state_pub_ = nh.advertise<moveit_msgs::DisplayRobotState>("ik_robot_state", 1, true);

  // init action clients
  joint_traj_ac_.reset(new FollowJointTrajectoryActionClient("joints/arm_trajectory_controller/follow_joint_trajectory", true));

  // init action servers
  fk_request_as_.reset(new FKRequestActionServer(nh, "fk_request", false));
  fk_request_as_->registerGoalCallback(boost::bind(&IKSolverNode::fkRequestActionGoalCb, this));
  fk_request_as_->start();

  ik_request_as_.reset(new IKRequestActionServer(nh, "ik_request", false));
  ik_request_as_->registerGoalCallback(boost::bind(&IKSolverNode::ikRequestActionGoalCb, this));
  ik_request_as_->start();
}

IKSolverNode::~IKSolverNode()
{
}

void IKSolverNode::update()
{
  if (auto_snap_to_current_pose_)
    snapToCurrentPose(visualization_msgs::InteractiveMarkerFeedbackConstPtr());

  // publish ik robot state
  moveit_msgs::DisplayRobotState robot_state_msg;
  robot_state_msg.state.joint_state.header.stamp = ros::Time::now();
  for (const std::pair<std::string, double>& e : ik_solver_.getIKResult())
  {
    robot_state_msg.state.joint_state.name.push_back(e.first);
    robot_state_msg.state.joint_state.position.push_back(e.second);
  }

  moveit_msgs::ObjectColor color;
  color.color.r = 1.0;
  color.color.g = 0.5;
  color.color.b = 0.0;
  color.color.a = 1.0;
  for (const std::string& name : link_names_)
  {
    color.id = name;
    robot_state_msg.highlight_links.push_back(color);
  }

  robot_state_pub_.publish(robot_state_msg);

  // add execute menu entry
  if (!has_ik_result_ && !ik_solver_.getIKResult().empty())
  {
    ik_marker_->setVisible(execute_handle_, true);
    has_ik_result_ = true;
  }

  // perform online ik planning if enabled
  if (online_ik_enabled_ && ik_marker_->isMoving() && !ik_request_as_->isActive())
    ik_solver_.ikRequest(ik_marker_->getPose(), boost::bind(&IKSolverNode::ikResultCb, this, _1));
}

void IKSolverNode::setOnlineIKEnabled(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  const interactive_markers::MenuHandler::EntryHandle& handle = feedback->menu_entry_id;
  if (ik_marker_->getCheckState(handle) == interactive_markers::MenuHandler::CHECKED)
  {
    ik_marker_->setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
    online_ik_enabled_ = false;
  }
  else
  {
    ik_marker_->setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
    online_ik_enabled_ = true;
  }

  ik_marker_->setVisible(ik_request_handle_, !online_ik_enabled_);
}

void IKSolverNode::setReuseIKSolution(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  const interactive_markers::MenuHandler::EntryHandle& handle = feedback->menu_entry_id;
  if (ik_marker_->getCheckState(handle) == interactive_markers::MenuHandler::CHECKED)
  {
    ik_marker_->setCheckState(handle, interactive_markers::MenuHandler::UNCHECKED);
    reuse_ik_solution_enabled_ = false;
  }
  else
  {
    ik_marker_->setCheckState(handle, interactive_markers::MenuHandler::CHECKED);
    reuse_ik_solution_enabled_ = true;
  }
}

void IKSolverNode::solveIK(const visualization_msgs::InteractiveMarkerFeedbackConstPtr /*feedback*/)
{
  if (!ik_request_as_->isActive())
    ik_solver_.ikRequest(ik_marker_->getPose(), boost::bind(&IKSolverNode::ikResultCb, this, _1));
}

void IKSolverNode::snapToCurrentPose(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  try
  {
    if (tf_listener_.canTransform(base_frame_, endeffector_frame_, ros::Time(0)))
    {
      // move ik marker
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(base_frame_, endeffector_frame_, ros::Time(0), transform);
      geometry_msgs::PoseStamped pose;
      tf::poseStampedTFToMsg(tf::Stamped<tf::Pose>(transform, transform.stamp_, transform.frame_id_), pose);
      ik_marker_->setPose(pose.pose, pose.header);

      // update initial configuration for IK solver
      if (last_joint_state_msg_)
      {
        std::map<std::string, double> initial_config;

        // get joint positions from joint states
        for (size_t i = 0; i < last_joint_state_msg_->name.size(); i++)
        {
          const std::string& name = last_joint_state_msg_->name[i];
          if (std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end())
            initial_config[name] = last_joint_state_msg_->position[i];
        }

        // check if list is complete
        if (initial_config.size() == joint_names_.size())
        {
          ik_solver_.setStartConfiguration(initial_config);
          auto_snap_to_current_pose_ = false;

          // trigger IK solver
          solveIK(feedback);
        }
        else
          ROS_WARN("[IKSolver] Couldn't update initial solver configuration as not all joints are defined by latest joint state message.");
      }
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[IKSolverNode] %s", ex.what());
  }
}

void IKSolverNode::executeIk(const visualization_msgs::InteractiveMarkerFeedbackConstPtr /*feedback*/)
{
  // check connection to action server
  if (!joint_traj_ac_->waitForServer(ros::Duration(1.0)))
  {
    ROS_ERROR("[IKNewton] Can't connect to joint trajectory server!");
    return;
  }

  // transform goal joint angles into a joint trajectory message
  const std::map<std::string, double>& ik_result = ik_solver_.getIKResult();

  control_msgs::FollowJointTrajectoryGoal goal;
  trajectory_msgs::JointTrajectory& trajectory = goal.trajectory;

  trajectory.joint_names = joint_names_;

  trajectory_msgs::JointTrajectoryPoint point;

  for (const std::string& joint_name : joint_names_)
  {
    std::map<std::string, double>::const_iterator itr = ik_result.find(joint_name);
    if (itr == ik_result.end())
    {
      ROS_ERROR("[IKNewton] Missing joint '%s' in IK solution!", itr->first.c_str());
      return;
    }
    else
      point.positions.push_back(itr->second);
  }

  double time = std::max(1.0, ik_solver_.getIKDistance() / endeffector_speed_);

  // compute duration based on cartesian distance
  point.time_from_start = ros::Duration(time*speed_scale_);

  trajectory.points.push_back(point);

  // send action goal
  joint_traj_ac_->sendGoal(goal);
}

void IKSolverNode::fkResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr result)
{
}

void IKSolverNode::ikResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr result)
{
  // use last ik solution as new initial configuration
  if (reuse_ik_solution_enabled_ && result->success)
    ik_solver_.setStartConfiguration(result->name, result->position);
}

void IKSolverNode::fkRequestActionGoalCb()
{
  // accept the new goal
  turtlebot3_exercise_msgs::FKRequestGoalConstPtr goal = fk_request_as_->acceptNewGoal();

  // check if new goal was preempted in the meantime
  if (fk_request_as_->isPreemptRequested())
  {
    fk_request_as_->setPreempted();
    return;
  }

  if (!ik_solver_.fkRequest(goal->request.name, goal->request.position, boost::bind(&IKSolverNode::fkRequestActionResultCb, this, _1)))
    fk_request_as_->setAborted();
}

void IKSolverNode::fkRequestActionResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr result)
{
  /// Warning: This implementation is not multi-request safe for simplicity
  // transform goal joint angles for action result
  if (fk_request_as_->isActive())
  {
    // send action result
    turtlebot3_exercise_msgs::FKRequestResult action_result;
    action_result.result = *result;
    fk_request_as_->setSucceeded(action_result);
  }

  fkResultCb(result);
}

void IKSolverNode::ikRequestActionGoalCb()
{
  // accept the new goal
  turtlebot3_exercise_msgs::IKRequestGoalConstPtr goal = ik_request_as_->acceptNewGoal();

  // check if new goal was preempted in the meantime
  if (ik_request_as_->isPreemptRequested())
  {
    ik_request_as_->setPreempted();
    return;
  }

  ik_marker_->setPose(goal->request.goal_pose.pose, goal->request.goal_pose.header);

  // set start configuration (weak consistency check)
  if (goal->request.name.size() == joint_names_.size())
    ik_solver_.setStartConfiguration(goal->request.name, goal->request.position);

  if (!ik_solver_.ikRequest(goal->request.goal_pose, boost::bind(&IKSolverNode::ikRequestActionResultCb, this, _1)))
    ik_request_as_->setAborted();
}

void IKSolverNode::ikRequestActionResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr result)
{
  /// Warning: This implementation is not multi-request safe for simplicity
  // transform goal joint angles for action result
  if (ik_request_as_->isActive())
  {
    // send action result
    turtlebot3_exercise_msgs::IKRequestResult action_result;
    action_result.result = *result;
    ik_request_as_->setSucceeded(action_result);
  }

  ikResultCb(result);
}

void IKSolverNode::jointStateCb(const sensor_msgs::JointState::ConstPtr joint_state_msg)
{
  last_joint_state_msg_ = joint_state_msg;
}
} // namespace



int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot3_ik_solver");

  ros::NodeHandle nh;

  turtlebot3::IKSolverNode node(nh);

  ros::Rate loop_rate(nh.param("ik/update_rate", 1.0));

  while (ros::ok())
  {
    ros::spinOnce();
    node.update();
    loop_rate.sleep();
  }

  return 0;
}
