#include "dock_server.h"

#include <geometry_msgs/Twist.h>

iqr::DockServer::DockServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string & server_name)
    : action_name_(server_name),
      nh_(nh),
      private_nh_(nh),
      move_base_client_("move_base", true),
      actionlib::ActionServer<caster_app::DockAction>(private_nh, server_name,
          boost::bind(&DockServer::GoalCallback, this, _1),
          boost::bind(&DockServer::CancelCallback, this, _1),
          false) {
  // goal_ = caster_app::DockGoal();
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);
}

void iqr::DockServer::Initialize() {
  ROS_INFO("waitting for move_base action server...");
  move_base_client_.waitForServer();
  ROS_INFO("move_base action server connected.");
  start();
}

void iqr::DockServer::MoveToDock() {
  float delta;

  tf::StampedTransform current_pose, last_pose;

  geometry_msgs::Twist cmd;
  cmd.linear.x = 0.05;

  tf::StampedTransform robot_pose;
  try{
   tf_listener_.lookupTransform("odom", "base_footprint",  
                            ros::Time(0), last_pose);
  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }

  while(delta > 1.0) {
    cmd_pub_.publish(cmd);

    try{
     tf_listener_.lookupTransform("word", "base_footprint",  
                              ros::Time(0), current_pose);
    }
    catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
    }

    delta = current_pose.getOrigin().distance(last_pose.getOrigin());
    ROS_INFO("delta: %f", delta);

    ros::Duration(0.02).sleep();
    ros::spinOnce();
  }

  ROS_INFO("stop robot");

  cmd.linear.x = 0;
  cmd_pub_.publish(cmd);

  goal_.setSucceeded(caster_app::DockResult(), "Docked");
}

void iqr::DockServer::MoveToDockReady() {
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = "world";
  mb_goal.target_pose.pose.position.x = 20.8905636619;
  mb_goal.target_pose.pose.position.y = 14.6787757997;
  mb_goal.target_pose.pose.position.z = 0.0;
  mb_goal.target_pose.pose.orientation.x = 0.0;
  mb_goal.target_pose.pose.orientation.y = 0.0;
  mb_goal.target_pose.pose.orientation.z = -0.0289632634423;
  mb_goal.target_pose.pose.orientation.w = 0.999580476685;

  move_base_client_.sendGoal(mb_goal,
            boost::bind(&iqr::DockServer::MovebaseDoneCallback, this, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&iqr::DockServer::MovebaseFeedbackCallback, this, _1));
}

void iqr::DockServer::MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  ROS_INFO("move_base finished in state [%s]", state.toString().c_str());
  
  caster_app::DockFeedback feedback;
  feedback.dock_feedback = "DockReady arrived";
  goal_.publishFeedback(feedback);

  MoveToDock();
}

void iqr::DockServer::MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  ROS_INFO_STREAM("Get move_base feedback" << ", x:" << feedback->base_position.pose.position.x << ", y:" << feedback->base_position.pose.position.y);

  tf::StampedTransform robot_pose;
  try{
   tf_listener_.lookupTransform("/world", "/base_footprint",  
                            ros::Time(0), robot_pose);
  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }

  tf::Vector3 dock_ready_pos(20.8905636619, 14.6787757997, 0.0);

  caster_app::DockFeedback ca_feedback;
  ca_feedback.dock_feedback = boost::str(boost::format("Moving to DockReady, %dm left") % dock_ready_pos.distance(robot_pose.getOrigin()));
  goal_.publishFeedback(ca_feedback);
}

void iqr::DockServer::GoalCallback(GoalHandle gh) {
  ROS_INFO("GoalCallback");
  goal_ = gh;
  caster_app::DockGoal goal = *goal_.getGoal();

  switch (goal.dock) {
    case true:
      goal_.setAccepted("Docking");
      MoveToDockReady();
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

  move_base_client_.cancelGoal();
}

// void iqr::DockServer::SetFeedback() {
//   ROS_INFO("ddd");
//     caster_app::DockFeedback fb;
//     fb.dock_feedback = "test";
//     goal_.publishFeedback(fb);
// }
