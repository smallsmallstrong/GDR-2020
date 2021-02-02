#include <turtlebot3_arm_controller_exercise/arm_controller.h>

#include <control_msgs/JointControllerState.h>
#include <iostream>



namespace turtlebot3
{
ArmController::ArmController(ros::NodeHandle& nh)
{
  // read parameters
  XmlRpc::XmlRpcValue val = nh.param("arm_controller/joints", XmlRpc::XmlRpcValue());

  std::string ns = ros::names::clean(ros::names::append("/gazebo", nh.getNamespace()));

  if (val.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < val.size(); i++)
    {
      const std::string& name = val[i]["name"];

      goal_position_sub_[name] = nh.subscribe<std_msgs::Float64>(ns + "/" + name + "_position/command", 1, boost::bind(&ArmController::goalPositionCb, this, name, _1));
      joint_effort_pub_[name] = nh.advertise<std_msgs::Float64>(ns + "/" + name + "_effort/command", 1, false);
      joint_controller_state_pub_[name] = nh.advertise<control_msgs::JointControllerState>(ns + "/" + name + "_effort/state", 1, false);

      p_[name] = val[i]["p"];
      i_[name] = val[i]["i"];
      d_[name] = val[i]["d"];

      max_effort_[name] = val[i]["max_effort"];

      current_position_[name] = 0.0;
      current_velocity_[name] = 0.0;
      goal_position_[name] = 0.0;
      sum_error_[name] = 0.0;
      last_dpos_[name] = 0.0;
      aw_[name] = 0.0;
      joint_names_.emplace_back(name);

      ROS_INFO("Added joint '%s with p=%f, i=%f, d=%f' ", name.c_str(), p_[name], i_[name], d_[name]);
    }
  }
  else
    ROS_ERROR("Couldn't find joint list on parameter server!");

  last_call_time_ = ros::Time::now();

  /// | Implement your code here |
  /// v                          v
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>("joint_states", 1, boost::bind(&ArmController::jointStateCb,this,_1));
  /// ^                          ^
  /// | -------- End ----------- |
}

ArmController::~ArmController()
{
}

void ArmController::update()
{
  ros::Time now = ros::Time::now();
  double delta_time = (now - last_call_time_).toSec();
  for (std::pair<std::string, double> entry : goal_position_)
  {
    const std::string &joint_name = entry.first;
    double effort_clamped = 0.0;
    double position_error = 0.0;
    updateJoint(joint_name, delta_time, effort_clamped, position_error);
    // publish joint effort command
    std_msgs::Float64 effort_msg;
    effort_msg.data = effort_clamped;
    joint_effort_pub_[joint_name].publish(effort_msg);

    // publish joint controller state
    control_msgs::JointControllerState state_msg;
    state_msg.header.stamp = now;
    state_msg.set_point = goal_position_[joint_name];
    state_msg.process_value = current_position_[joint_name];
    state_msg.process_value_dot = current_velocity_[joint_name];
    state_msg.error = position_error;
    state_msg.time_step = delta_time;
    state_msg.command = effort_clamped;
    state_msg.p = p_[joint_name];
    state_msg.i = i_[joint_name];
    state_msg.d = d_[joint_name];
    state_msg.i_clamp = 0.0;
    state_msg.antiwindup = false;
    joint_controller_state_pub_[joint_name].publish(state_msg);
  }
  last_call_time_ = now;
}

void ArmController::updateJoint(const std::string &joint_name,
                                double delta_time, double &effort_clamped,
                                double &position_error) {
  /// | Implement your code here |
  /// v                          v
    //b) calculate the error(from goal to current)(Eingang)
    position_error = goal_position_[joint_name]-current_position_[joint_name];
    sum_error_[joint_name]+=position_error*delta_time*0.5;
    double v_e = 0 - current_velocity_[joint_name];
    //b) calculate and update U or I(Ausgang)
    effort_clamped = p_[joint_name]*position_error+i_[joint_name]*sum_error_[joint_name]+d_[joint_name]*v_e;
    aw_[joint_name] = 1/i_[joint_name];
    //c) anti-windup strategy
    std::cout<<"effort"<<effort_clamped<<std::endl;
    std::cout<<"max_effort"<<max_effort_[joint_name]<<std::endl;
/*
    while(1){
        if(effort_clamped <= max_effort_[joint_name] && effort_clamped>=-max_effort_[joint_name]){
            break;
        }else if(effort_clamped > max_effort_[joint_name]){
            sum_error_[joint_name] = sum_error_[joint_name] + aw_[joint_name]*(-effort_clamped+max_effort_[joint_name]);
            effort_clamped = p_[joint_name]*position_error+i_[joint_name]*(sum_error_[joint_name])+d_[joint_name]*v_e;
        }else{
            sum_error_[joint_name] = sum_error_[joint_name] + aw_[joint_name]*(effort_clamped-max_effort_[joint_name]);
            effort_clamped = p_[joint_name]*position_error+i_[joint_name]*(sum_error_[joint_name])+d_[joint_name]*v_e;
        }

    }
*/
  /// ^                          ^
  /// | -------- End ----------- |
}

void ArmController::jointStateCb(const sensor_msgs::JointState::ConstPtr joint_state_msg)
{
  /// | Implement your code here |
  /// v                          v

    for (int i=0; i<joint_state_msg->position.size(); i++) {
        current_position_[joint_state_msg->name.at(i)] = joint_state_msg->position.at(i);
        current_velocity_[joint_state_msg->name.at(i)] = joint_state_msg->velocity.at(i);
    }

  /// ^                          ^
  /// | -------- End ----------- |
}

void ArmController::goalPositionCb(const std::string& joint_name, const std_msgs::Float64::ConstPtr goal_msg)
{
  /// | Implement your code here |
  /// v                          v

    //a) transfer goal_msg to goal_position_sub_
     goal_position_[joint_name]=goal_msg->data;

  /// ^                          ^
  /// | -------- End ----------- |
}
} // namespace
