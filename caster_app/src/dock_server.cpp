#include "dock_server.h"

iqr::DockServer::DockServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string & server_name)
    : action_name_(server_name),
      nh_(nh),
      private_nh_(private_nh),
      move_base_client_("move_base", true),
      actionlib::ActionServer<caster_app::DockAction>(private_nh, server_name,
          boost::bind(&DockServer::GoalCallback, this, _1),
          boost::bind(&DockServer::CancelCallback, this, _1),
          false) {
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("robot0/cmd_vel", 1000);

  private_nh_.param<float>("dock/dock_speed", dock_speed_, 0.05);
  private_nh_.param<float>("dock/dock_distance", dock_distance_, 1.0);
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<std::string>("odom_frame", odom_frame_, "odom");
  private_nh_.param<std::string>("base_frame", base_frame_, "base_footprint");

  ROS_INFO_STREAM("param: dock_spped " << dock_speed_ << ", dock_distance " << dock_distance_);
  ROS_INFO_STREAM("param: map_frame " << map_frame_ << ", odom_frame " << odom_frame_ << ", base_frame " << base_frame_);

  dock_ready_pose_.position.x = 20.8905636619;
  dock_ready_pose_.position.y = 14.6787757997;
  dock_ready_pose_.position.z = 0.0;
  dock_ready_pose_.orientation.x = 0.0;
  dock_ready_pose_.orientation.y = 0.0;
  dock_ready_pose_.orientation.z = -0.0289632634423;
  dock_ready_pose_.orientation.w = 0.999580476685;

  docked_ = false;
}

void iqr::DockServer::Initialize() {
  ROS_INFO("waitting for move_base action server...");
  move_base_client_.waitForServer();
  ROS_INFO("move_base action server connected.");
  start();
}

void iqr::DockServer::UnDock() {
  float delta = 0;

  tf::StampedTransform current_pose, last_pose;

  geometry_msgs::Twist cmd;
  cmd.linear.x = dock_speed_ * -1.0;

  tf::StampedTransform robot_pose;
  try{
    tf_listener_.lookupTransform(odom_frame_, base_frame_,
        ros::Time(0), last_pose);
  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }

  while(delta < dock_distance_) {
    cmd_pub_.publish(cmd);

    try{
      tf_listener_.lookupTransform(odom_frame_, base_frame_,
          ros::Time(0), current_pose);
    }
    catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
    }

    delta = current_pose.getOrigin().distance(last_pose.getOrigin());
    // ROS_INFO("delta: %f", delta);
    feedback_.dock_feedback = boost::str(boost::format("Moving to Dock, %dm left") % (dock_distance_-delta));
    goal_.publishFeedback(feedback_);

    ros::Duration(0.05).sleep();
    ros::spinOnce();
  }

  feedback_.dock_feedback = "Stop on DockReady";
  goal_.publishFeedback(feedback_);
  // ROS_INFO("stop robot");

  cmd.linear.x = 0;
  cmd_pub_.publish(cmd);

  docked_ = false;
  goal_.setSucceeded(caster_app::DockResult(), "Undocked");
  ROS_INFO("Undocked");
}

void iqr::DockServer::MoveToDock() {
  float delta = 0;

  tf::StampedTransform current_pose, last_pose;

  geometry_msgs::Twist cmd;
  cmd.linear.x = dock_speed_;

  tf::StampedTransform robot_pose;
  try{
    tf_listener_.lookupTransform(odom_frame_, base_frame_,
        ros::Time(0), last_pose);
  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }

  while(delta < dock_distance_) {
    cmd_pub_.publish(cmd);

    try{
      tf_listener_.lookupTransform(odom_frame_, base_frame_,
          ros::Time(0), current_pose);
    }
    catch (tf::TransformException ex){
     ROS_ERROR("%s",ex.what());
     ros::Duration(1.0).sleep();
    }

    delta = current_pose.getOrigin().distance(last_pose.getOrigin());
    // ROS_INFO("delta: %f", delta);
    feedback_.dock_feedback = boost::str(boost::format("Moving to Dock, %dm left") % (dock_distance_-delta));
    goal_.publishFeedback(feedback_);

    ros::Duration(0.05).sleep();
    ros::spinOnce();
  }

  feedback_.dock_feedback = "Stop on Dock";
  goal_.publishFeedback(feedback_);
  // ROS_INFO("stop robot");

  cmd.linear.x = 0;
  cmd_pub_.publish(cmd);

  docked_ = true;
  goal_.setSucceeded(caster_app::DockResult(), "Docked");
  ROS_INFO("Docked");
}

void iqr::DockServer::MoveToDockReady() {
  move_base_msgs::MoveBaseGoal mb_goal;
  mb_goal.target_pose.header.stamp = ros::Time::now();
  mb_goal.target_pose.header.frame_id = map_frame_;
  mb_goal.target_pose.pose = dock_ready_pose_;

  move_base_client_.sendGoal(mb_goal,
            boost::bind(&iqr::DockServer::MovebaseDoneCallback, this, _1, _2),
            actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
            boost::bind(&iqr::DockServer::MovebaseFeedbackCallback, this, _1));
}

void iqr::DockServer::MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result) {
  // ROS_INFO("move_base finished in state [%s]", state.toString().c_str());
  
  caster_app::DockFeedback feedback;
  feedback.dock_feedback = "DockReady arrived";
  goal_.publishFeedback(feedback);

  ros::Duration(2.0).sleep();
  MoveToDock();
}

void iqr::DockServer::MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) {
  // ROS_INFO_STREAM("Get move_base feedback" << ", x:" << feedback->base_position.pose.position.x << ", y:" << feedback->base_position.pose.position.y);

  tf::StampedTransform robot_pose;
  try{
    tf_listener_.lookupTransform(map_frame_, base_frame_,  
        ros::Time(0), robot_pose);
  }
  catch (tf::TransformException ex){
   ROS_ERROR("%s",ex.what());
   ros::Duration(1.0).sleep();
  }

  tf::Vector3 dock_ready_pos(dock_ready_pose_.position.x, dock_ready_pose_.position.y, dock_ready_pose_.position.z);

  caster_app::DockFeedback ca_feedback;
  ca_feedback.dock_feedback = boost::str(boost::format("Moving to DockReady, %dm left") % dock_ready_pos.distance(robot_pose.getOrigin()));
  goal_.publishFeedback(ca_feedback);
}

void iqr::DockServer::GoalCallback(GoalHandle gh) {
  // ROS_INFO("GoalCallback");
  goal_ = gh;
  caster_app::DockGoal goal = *goal_.getGoal();

  switch (goal.dock) {
    case true:
      if (docked_ == true) {
        goal_.setRejected(caster_app::DockResult(), "already docked");
      } else {
        ROS_INFO("Docking");
        goal_.setAccepted("Docking");
        MoveToDockReady();
      }
      break;
    case false:
      if(docked_ == false) {
        goal_.setRejected(caster_app::DockResult(), "not on dock");
      } else {
        ROS_INFO("Start undock");
        goal_.setAccepted("Start undock");
        UnDock();
      }
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