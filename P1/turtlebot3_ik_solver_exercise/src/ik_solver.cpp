#include <turtlebot3_ik_solver_exercise/ik_solver.h>



namespace turtlebot3
{
IKSolver::IKSolver(ros::NodeHandle& nh)
  : uid_(0)
{
  // read parameters
  base_frame_ = nh.param("ik/base_frame", std::string("base_link"));
  endeffector_frame_ = nh.param("ik/endeffector_frame", std::string("gripper_link"));

  max_solving_time_ = nh.param("ik/max_solving_time", 0);
  max_eps_ = nh.param("ik/max_eps_", 1.0e-3);

  for (std::string s : nh.param("ik/joints", std::vector<std::string>()))
    ik_joint_positions_[s] = 0.0;

  // init topics
  fk_result_sub_ = nh.subscribe("fk_result", 1, &IKSolver::fkResultCb, this);
  ik_result_sub_ = nh.subscribe("ik_result", 1, &IKSolver::ikResultCb, this);
  fk_request_pub_ = nh.advertise<turtlebot3_exercise_msgs::FKRequest>("fk_request", 1, true);
  ik_request_pub_ = nh.advertise<turtlebot3_exercise_msgs::IKRequest>("ik_request", 1, true);
}

IKSolver::~IKSolver()
{
}

void IKSolver::setStartConfiguration(const std::map<std::string, double>& joints)
{
  initial_joint_positions_ = joints;
}

void IKSolver::setStartConfiguration(const std::vector<std::string>& name, const std::vector<double>& position)
{
  if (name.size() != position.size())
  {
    ROS_ERROR("[IKSolver] setStartConfiguration was called with different sizes of names and positions.");
    return;
  }

  initial_joint_positions_.clear();
  for (size_t i = 0; i < name.size(); i++)
    initial_joint_positions_[name[i]] = position[i];
}

bool IKSolver::fkRequest(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions, FKResultCallback result_cb) const
{
  if (joint_names.size() != joint_positions.size())
  {
    ROS_ERROR("[IKSolver] Illegal FK request. Joint names list size must be the same size of given position vector.");
    return false;
  }

  turtlebot3_exercise_msgs::FKRequest req;
  req.name = joint_names;
  req.position = joint_positions;

  return fkRequest(req, result_cb);
}

bool IKSolver::fkRequest(const turtlebot3_exercise_msgs::FKRequest& req, FKResultCallback result_cb) const
{
  turtlebot3_exercise_msgs::FKRequest msg = req;
  msg.uid = ++uid_;

  fk_result_cb_ = result_cb;

  fk_request_pub_.publish(msg);

  return true;
}

bool IKSolver::ikRequest(const geometry_msgs::Pose& goal_pose, const std_msgs::Header& header, IKResultCallback result_cb) const
{
  geometry_msgs::PoseStamped pose;
  pose.header = header;
  pose.pose = goal_pose;
  return ikRequest(pose, result_cb);
}

bool IKSolver::ikRequest(const geometry_msgs::PoseStamped& goal_pose, IKResultCallback result_cb) const
{
  turtlebot3_exercise_msgs::IKRequest req;

  // check if transform is required
  if (!goal_pose.header.frame_id.empty() && goal_pose.header.frame_id != base_frame_)
  {
    try
    {
      if (tf_listener_.canTransform(base_frame_, goal_pose.header.frame_id, ros::Time(0)))
      {
        tf::Stamped<tf::Pose> pose;
        tf::poseStampedMsgToTF(goal_pose, pose);
        tf_listener_.transformPose(base_frame_, pose, pose);
        tf::poseStampedTFToMsg(pose, req.goal_pose);
      }
      else
      {
        ROS_ERROR("[IKNewton] Unknown transformation from '%s' to '%s'.", base_frame_.c_str(), goal_pose.header.frame_id.c_str());
        return false;
      }
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("[IKNewton] %s", ex.what());
      return false;
    }
  }
  else
  {
    req.goal_pose = goal_pose;
  }

  // set initial joint configuration
  for (const std::pair<std::string, double>& e : initial_joint_positions_)
  {
    req.name.push_back(e.first);
    req.position.push_back(e.second);
  }

  // set solver parameters
  req.max_solving_time = max_solving_time_;
  req.max_eps = max_eps_;

  return ikRequest(req, result_cb);
}

bool IKSolver::ikRequest(const turtlebot3_exercise_msgs::IKRequest& req, IKResultCallback result_cb) const
{
  turtlebot3_exercise_msgs::IKRequest msg = req;
  msg.uid = ++uid_;

  ik_result_cb_ = result_cb;

  // compute cartesian travel distance
  tf::StampedTransform current_pose;
  try
  {
    if (tf_listener_.canTransform(base_frame_, endeffector_frame_, ros::Time(0)))
    {
      tf_listener_.lookupTransform(base_frame_, endeffector_frame_, ros::Time(0), current_pose);
    }
    else
    {
      ROS_ERROR("[IKNewton] Unknown transformation from '%s' to '%s'.", base_frame_.c_str(), endeffector_frame_.c_str());
      return false;
    }
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR("[IKNewton] %s", ex.what());
    return false;
  }

  tf::Stamped<tf::Pose> goal_pose;
  tf::poseStampedMsgToTF(req.goal_pose, goal_pose);
  ik_distance_ = tf::tfDistance(current_pose.getOrigin(), goal_pose.getOrigin());

  ik_request_pub_.publish(msg);

  return true;
}

const std::map<std::string, double>& IKSolver::getIKResult() const
{
  return ik_joint_positions_;
}

double IKSolver::getIKDistance() const
{
  return ik_distance_;
}

void IKSolver::fkResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr msg)
{
  /// @TODO: Check uid
  if (fk_result_cb_)
    fk_result_cb_(msg);
}

void IKSolver::ikResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr msg)
{
  /// @TODO: Check uid
  if (msg->success)
  {
    // store solution
    for (size_t i = 0; i < msg->name.size(); i++)
      ik_joint_positions_[msg->name[i]] = msg->position[i];
  }

  if (ik_result_cb_)
    ik_result_cb_(msg);
}
} // namespace
