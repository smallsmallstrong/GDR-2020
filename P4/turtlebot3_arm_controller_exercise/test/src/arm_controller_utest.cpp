#include <gtest/gtest.h>

#include <turtlebot3_arm_controller_exercise/arm_controller_node.h>

namespace turtlebot3 {
namespace {

class ArmControllerTestable : public ArmController {
public:
  ArmControllerTestable(ros::NodeHandle &nh) : ArmController(nh) {}

  void jointStateCb(const sensor_msgs::JointState::ConstPtr joint_state_msg) {
    ArmController::jointStateCb(joint_state_msg);
  }

  void goalPositionCb(const std::string &joint_name,
                      const std_msgs::Float64::ConstPtr goal_msg) {
    ArmController::goalPositionCb(joint_name, goal_msg);
  }

  const std::map<std::string, double> &getCurrentPosition() const {
    return current_position_;
  }
  const std::map<std::string, double> &getCurrentVelocity() const {
    return current_velocity_;
  }
  const std::map<std::string, double> &getGoalPosition() const {
    return goal_position_;
  }

  const std::map<std::string, double> &getMaxEffort() const{
    return max_effort_;
  }

  const bool setPID(const std::string& joint_name, double p, double i, double d){
    if(p_.find(joint_name)==p_.end())return false;
    p_[joint_name]=p;
    i_[joint_name]=i;
    d_[joint_name]=d;
    return true;
  }

  const bool setPID(const std::string& joint_name, double p, double i, double d, double max_effort){
    if(!setPID(joint_name, p,i,d))return false;
    max_effort_[joint_name] = max_effort;
    return true;
  }
  const std::vector<std::string> &getJointNames() const { return joint_names_; }
};

class ArmControllerTest : public ::testing::Test {
protected:
  ArmControllerTest() : nh_(), controller_(nh_) {}
  ros::NodeHandle nh_;
  ArmControllerTestable controller_;
};

TEST_F(ArmControllerTest, jointStateCb) {
  sensor_msgs::JointStatePtr target_joint_state(new sensor_msgs::JointState);
  std::vector<std::string> joint_names = controller_.getJointNames();
  size_t joint_idx = 1;
  for (const std::string &joint_name : joint_names) {
    target_joint_state->name.emplace_back(joint_name);
    target_joint_state->position.emplace_back(M_PI / joint_idx);
    target_joint_state->velocity.emplace_back(M_PI * joint_idx);
    ++joint_idx;
  }

  controller_.jointStateCb(target_joint_state);
  std::map<std::string, double> updated_positions =
      controller_.getCurrentPosition();
  std::map<std::string, double> updated_velocities =
      controller_.getCurrentVelocity();

  for (size_t joint_idx = 0; joint_idx < joint_names.size(); ++joint_idx) {
    const std::string &joint_name = joint_names[joint_idx];
    EXPECT_FLOAT_EQ(target_joint_state->position[joint_idx],
                    updated_positions[joint_name]);
    EXPECT_FLOAT_EQ(target_joint_state->velocity[joint_idx],
                    updated_velocities[joint_name]);
  }
}

TEST_F(ArmControllerTest, goalPositionCb) {
  double target_goal = 0.42;
  std::string joint_name = controller_.getJointNames()[2];
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  std::map<std::string, double> updated_goal_positions =
      controller_.getGoalPosition();
  EXPECT_FLOAT_EQ(target_goal, updated_goal_positions[joint_name]);
}

TEST_F(ArmControllerTest, updateFromZeroVelocity) {
  double target_goal = 0.01;
  std::string joint_name = controller_.getJointNames()[2];
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  controller_.updateJoint(joint_name, 1.0, effort, position_error);
  EXPECT_NEAR(effort, 0.7, 2.5e-1); // Results differ depending on integration scheme
  EXPECT_FLOAT_EQ(position_error, 0.01);
}

TEST_F(ArmControllerTest, updateFromMotion) {
  sensor_msgs::JointStatePtr target_joint_state(new sensor_msgs::JointState);
  std::vector<std::string> joint_names = controller_.getJointNames();
  size_t joint_idx = 1;
  for (const std::string &joint_name : joint_names) {
    target_joint_state->name.emplace_back(joint_name);
    target_joint_state->position.emplace_back(0.1 * joint_idx);
    target_joint_state->velocity.emplace_back(-0.02 * joint_idx);
    ++joint_idx;
  }
  controller_.jointStateCb(target_joint_state);
  double target_goal = 0.09;
  std::string joint_name = controller_.getJointNames()[0];
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  controller_.updateJoint(joint_name, 1.0, effort, position_error);
  EXPECT_NEAR(effort, -0.68, 2.5e-1); // Results differ depending on integration scheme
  EXPECT_FLOAT_EQ(position_error, -0.01);
}

TEST_F(ArmControllerTest, clamping) {
  double target_goal = 1000;
  std::string joint_name = controller_.getJointNames()[2];
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  controller_.updateJoint(joint_name, 1000.0, effort, position_error);
  std::map<std::string, double> max_efforts =
      controller_.getMaxEffort();
  double max_effort = max_efforts[joint_name];
  EXPECT_NEAR(std::abs(effort), std::abs(max_effort), 2.5e-1); 
}

TEST_F(ArmControllerTest, anti_windup) {
  double target_goal = 0.1;
  std::string joint_name = controller_.getJointNames()[2];
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  for(int i = 0; i < 1000;i++){
    controller_.updateJoint(joint_name, 0.01, effort, position_error); //try to increase the error integral
  }
  
  target_goal_msg->data = -0.01; // reverse the target and therefore also the sign of the error, this should decrease the integral
  controller_.goalPositionCb(joint_name, target_goal_msg);

  controller_.updateJoint(joint_name, 0.01, effort, position_error);
  double effort2 = 0.0;
  controller_.updateJoint(joint_name, 0.01, effort2, position_error);//if anti windup is working the total effort should directly decrease
  EXPECT_LT(effort2, effort);
  
  printf("erro1, erro2: %f, %f", effort, effort2);
}

TEST_F(ArmControllerTest, proportional_only) {
  std::string joint_name = controller_.getJointNames()[2];
  controller_.setPID(joint_name, 0.5,0,0,1);
  double target_goal = 0.5;
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  controller_.updateJoint(joint_name, 1.0, effort, position_error);
  EXPECT_FLOAT_EQ(effort, 0.25); // Results differ depending on integration scheme
  EXPECT_FLOAT_EQ(position_error, 0.5);
}

TEST_F(ArmControllerTest, differential_only) {
  std::string joint_name = controller_.getJointNames()[2];
  controller_.setPID(joint_name, 0,0,2,1);
  sensor_msgs::JointStatePtr target_joint_state(new sensor_msgs::JointState);

  
  target_joint_state->name.emplace_back(joint_name);
  target_joint_state->position.emplace_back(0.5);
  target_joint_state->velocity.emplace_back(0.3);
  
  controller_.jointStateCb(target_joint_state);
  double target_goal = 0.09;
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  controller_.updateJoint(joint_name, 1.0, effort, position_error);
  EXPECT_FLOAT_EQ(effort, -0.6); // Results differ depending on integration scheme
}

TEST_F(ArmControllerTest, integral_only) {
  std::string joint_name = controller_.getJointNames()[2];
  controller_.setPID(joint_name, 0,0.5,0,1);
  sensor_msgs::JointStatePtr target_joint_state(new sensor_msgs::JointState);

  
  target_joint_state->name.emplace_back(joint_name);
  target_joint_state->position.emplace_back(0.0);
  target_joint_state->velocity.emplace_back(0.0);
  
  controller_.jointStateCb(target_joint_state);
  double target_goal = 0.01;
  std_msgs::Float64::Ptr target_goal_msg(new std_msgs::Float64);
  target_goal_msg->data = target_goal;
  controller_.goalPositionCb(joint_name, target_goal_msg);
  double effort = 0.0;
  double position_error = 0.0;
  for (size_t i = 0; i < 7; i++)
  {
    controller_.updateJoint(joint_name, 0.5, effort, position_error);
  }

  EXPECT_NEAR(effort, 0.0175,1e-2); // Results differ depending on integration scheme
}


} // namespace
} // namespace turtlebot3
