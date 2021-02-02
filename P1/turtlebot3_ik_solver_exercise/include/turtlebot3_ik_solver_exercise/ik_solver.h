#ifndef TURTLEBOT3_IK_NEWTON_H__
#define TURTLEBOT3_IK_NEWTON_H__

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <turtlebot3_exercise_msgs/FKRequest.h>
#include <turtlebot3_exercise_msgs/IKRequest.h>
#include <turtlebot3_exercise_msgs/FKResult.h>
#include <turtlebot3_exercise_msgs/IKResult.h>



namespace turtlebot3
{
class IKSolver
{
public:
  typedef boost::function<void (const turtlebot3_exercise_msgs::FKResultConstPtr)> FKResultCallback;
  typedef boost::function<void (const turtlebot3_exercise_msgs::IKResultConstPtr)> IKResultCallback;

  IKSolver(ros::NodeHandle& nh);
  virtual ~IKSolver();

  /**
   * @brief Sets seeds of solver (start/current joint configuration)
   * @param joints Current joint configuration
   */
  void setStartConfiguration(const std::map<std::string, double>& joints);

  /**
   * @brief Sets seeds of solver (start/current joint configuration)
   * @param name List of joint names
   * @param position List of corresponding joint positions
   */
  void setStartConfiguration(const std::vector<std::string>& name, const std::vector<double>& position);

  /**
   * @brief Computes forward kinematics of endeffector
   * @param joint_names joint names
   * @param joint_positions joint positions
   * @param result_cb Optional callback when solution has arrived
   * @return true if request call was successful (not indicating successful FK soltion!)
   */
  bool fkRequest(const std::vector<std::string>& joint_names, const std::vector<double>& joint_positions, FKResultCallback result_cb = FKResultCallback()) const;

  /**
   * @brief Computes forward kinematics of endeffector
   * @param req FK request
   * @param result_cb Optional callback when solution has arrived
   * @return true if request call was successful (not indicating successful FK soltion!)
   */
  bool fkRequest(const turtlebot3_exercise_msgs::FKRequest& req, FKResultCallback result_cb = FKResultCallback()) const;

  /**
   * @brief Calling this functions triggers IK solver to compute joint angles in order
   * to get the endeffector to the desired goal pose.
   * @param goal_pose target pose for endeffector
   * @param header optional header for given pose
   * @param result_cb Optional callback when solution has arrived
   * @return true if request call was successful (not indicating successful IK soltion!)
   */
  bool ikRequest(const geometry_msgs::Pose& goal_pose, const std_msgs::Header& header = std_msgs::Header(), IKResultCallback result_cb = IKResultCallback()) const;

  /**
   * @brief Calling this functions triggers IK solver to compute joint angles in order
   * to get the endeffector to the desired goal pose.
   * @param goal_pose target pose for endeffector
   * @param result_cb Optional callback when solution has arrived
   * @return true if request call was successful (not indicating successful IK soltion!)
   */
  bool ikRequest(const geometry_msgs::PoseStamped& goal_pose, IKResultCallback result_cb = IKResultCallback()) const;

  /**
   * @brief Calling this functions triggers IK solver to compute joint angles in order
   * to get the endeffector to the desired goal pose.
   * @param req IK solver request
   * @param result_cb Optional callback when solution has arrived
   * @return true if request call was successful (not indicating successful IK soltion!)
   */
  bool ikRequest(const turtlebot3_exercise_msgs::IKRequest& req, IKResultCallback result_cb = IKResultCallback()) const;

  /**
   * @brief Returns last ik result
   * @return last ik result
   */
  const std::map<std::string, double>& getIKResult() const;

  /**
   * @brief Returns cartesian travel distance of endeffector
   * @return travel distance in [m]
   */
  double getIKDistance() const;

protected:
  void fkResultCb(const turtlebot3_exercise_msgs::FKResultConstPtr msg);
  void ikResultCb(const turtlebot3_exercise_msgs::IKResultConstPtr msg);

  mutable uint64_t uid_;

  std::string base_frame_;
  std::string endeffector_frame_;

  uint64_t max_solving_time_;
  double max_eps_;

  std::map<std::string, double> initial_joint_positions_;
  std::map<std::string, double> ik_joint_positions_;

  mutable double ik_distance_;

  mutable FKResultCallback fk_result_cb_;
  mutable IKResultCallback ik_result_cb_;

  tf::TransformListener tf_listener_;

  // subscriber/publisher for Octave Note
  ros::Subscriber fk_result_sub_;
  ros::Subscriber ik_result_sub_;
  ros::Publisher fk_request_pub_;
  ros::Publisher ik_request_pub_;
};
}

#endif
