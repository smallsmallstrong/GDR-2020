#include <turtlebot3_ik_solver_exercise/ik_marker.h>

#include <tf/tf.h>



namespace turtlebot3
{
IKMarker::IKMarker(const std::string& topic, const std::string& base_frame, double marker_scale)
  : is_moving_(false)
{
  server_.reset(new interactive_markers::InteractiveMarkerServer(topic, "", false));

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = base_frame;
  int_marker.pose.orientation.w = 1.0;
  int_marker.scale = marker_scale;

  int_marker.name = INT_MARKER_NAME;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;

  /// init ghost hand
  visualization_msgs::Marker marker;
  //marker.type = visualization_msgs::Marker::SPHERE;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_resource = "package://tuda_turtlebot3_arm_description/meshes/gripper/gripper_base.stl";
  marker.pose.position.x = -0.14;
  marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI, -M_PI, 0.0);
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.color.r = 0.1;
  marker.color.g = 0.8;
  marker.color.b = 0.1;
  marker.color.a = 1.0;
  control.markers.push_back(marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  int_marker.controls.push_back(control);

  /// init moving axes
  control = visualization_msgs::InteractiveMarkerControl();

  // x-axis
  control.orientation.x = 0.707;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // y-axis
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0.707;
  control.orientation.w = 0.707;
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  // z-axis
  control.orientation.x = 0;
  control.orientation.y = 0.707;
  control.orientation.z = 0;
  control.orientation.w = 0.707;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  /// Menu handler
  marker = visualization_msgs::Marker();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = int_marker.scale * 0.1;
  marker.scale.y = int_marker.scale * 0.1;
  marker.scale.z = int_marker.scale * 0.1;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  control = visualization_msgs::InteractiveMarkerControl();
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.always_visible = true;
  control.description = "Options";
  control.markers.push_back(marker);
  int_marker.controls.push_back(control);

  // init interactive server marker
  server_->insert(int_marker);
  server_->setCallback(int_marker.name, boost::bind(&IKMarker::processFeedback, this, _1));

  menu_handler_.apply(*server_, INT_MARKER_NAME);

  server_->applyChanges();
}

interactive_markers::MenuHandler::EntryHandle IKMarker::insertMenuItem(const std::string& title, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb)
{
  interactive_markers::MenuHandler::EntryHandle handle = menu_handler_.insert(title, feedback_cb);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

interactive_markers::MenuHandler::EntryHandle IKMarker::insertCheckableMenuItem(const std::string& title, const interactive_markers::MenuHandler::CheckState& state, const interactive_markers::MenuHandler::FeedbackCallback& feedback_cb)
{
  interactive_markers::MenuHandler::EntryHandle handle = menu_handler_.setCheckState(menu_handler_.insert(title, feedback_cb), state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
  return handle;
}

interactive_markers::MenuHandler::CheckState IKMarker::getCheckState(const interactive_markers::MenuHandler::EntryHandle& handle) const
{
  interactive_markers::MenuHandler::CheckState state;
  menu_handler_.getCheckState(handle, state);
  return state;
}

void IKMarker::setCheckState(const interactive_markers::MenuHandler::EntryHandle& handle, const interactive_markers::MenuHandler::CheckState& state)
{
  menu_handler_.setCheckState(handle, state);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void IKMarker::setVisible(const interactive_markers::MenuHandler::EntryHandle& handle, bool visible)
{
  menu_handler_.setVisible(handle, visible);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

IKMarker::~IKMarker()
{
  server_->clear();
}

void IKMarker::setPose(const geometry_msgs::Pose& pose, const std_msgs::Header& header)
{
  server_->setPose(INT_MARKER_NAME, pose);
  server_->applyChanges();
  pose_.header = header;
  pose_.pose = pose;
}

const geometry_msgs::PoseStamped& IKMarker::getPose() const
{
  return pose_;
}

void IKMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr feedback)
{
  //ROS_INFO_STREAM(*feedback);

  // only trigger on mouse release
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
    is_moving_ = false;
  else
    is_moving_ = true;

  // constrain rotation
  tf::Pose p;
  tf::poseMsgToTF(feedback->pose, p);

  tf::Vector3 tcp_direction = tf::Matrix3x3(p.getRotation()) * tf::Vector3(1.0, 0.0, 0.0);

  double yaw = atan2(p.getOrigin().getY(), p.getOrigin().getX());
  tf::Matrix3x3 rotation_z(tf::createQuaternionFromYaw(-yaw));
  tf::Vector3 tcp_direction_yaw_removed = rotation_z * tcp_direction;

  double pitch = atan2(tcp_direction_yaw_removed.getX(), tcp_direction_yaw_removed.getZ()) - M_PI_2;

  p.setRotation(tf::createQuaternionFromRPY(M_PI_2, pitch, yaw));

  // update
  pose_.header = feedback->header;
  tf::poseTFToMsg(p, pose_.pose);

  setPose(pose_.pose, pose_.header);
}
} // namespace
