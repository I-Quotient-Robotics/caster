#include "dock_server.h"

iqr::DockServer::DockServer(ros::NodeHandle &nh, const std::string & server_name)
    : action_name_(server_name),
      private_nh_(nh),
      actionlib::ActionServer<caster_app::DockAction>(nh, server_name,
          boost::bind(&DockServer::GoalCallback, this, _1),
          boost::bind(&DockServer::CancelCallback, this, _1),
          false) {
  // goal_ = caster_app::DockGoal();
}

void iqr::DockServer::Initialize() {
  start();
}

void iqr::DockServer::GoalCallback(GoalHandle gh) {
  ROS_INFO("GoalCallback");
  goal_ = gh;
  caster_app::DockGoal goal = *gh.getGoal();

  switch (goal.dock) {
    case true:
      goal_.setAccepted("Start docking");
      // gh.setSucceeded(caster_app::DockResult(), "The ref server has succeeded");
      break;
    case false:
      goal_.setAccepted("Start undock");
      // goal_.setAborted(caster_app::DockResult(), "The ref server has aborted");
      break;
    default:
      ROS_WARN_NAMED(action_name_, "unknown dock data type, should be true or false");
      break;
  }
}

void iqr::DockServer::CancelCallback(GoalHandle) {
  ROS_INFO("%s cancel has been called", action_name_.c_str());
}

// void iqr::DockServer::SetFeedback() {
//   ROS_INFO("ddd");
//     caster_app::DockFeedback fb;
//     fb.dock_feedback = "test";
//     goal_.publishFeedback(fb);
// }
