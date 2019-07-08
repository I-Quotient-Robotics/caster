#ifndef DOCK_SERVER_H_
#define DOCK_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <caster_app/DockAction.h>

namespace iqr {
class DockServer : public actionlib::ActionServer<caster_app::DockAction> {
  private:
    bool docked_;

    std::string action_name_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    GoalHandle goal_;

    caster_app::DockResult result_;
    caster_app::DockFeedback feedback_;

    tf::TransformListener tf_listener_;

    ros::Publisher cmd_pub_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;


    /* parameter */
    float dock_speed_;
    float dock_distance_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;

    geometry_msgs::Pose dock_ready_pose_;

    void GoalCallback(GoalHandle gh);
    void CancelCallback(GoalHandle gh);

    // move_base action client callback
    void MovebaseFeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);
    void MovebaseDoneCallback(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result);

    void UnDock();
    void MoveToDock();
    void MoveToDockReady();

  public:
    typedef actionlib::ServerGoalHandle<caster_app::DockAction> GoalHandle;

    DockServer(ros::NodeHandle &nh, ros::NodeHandle &private_nh, const std::string & server_name);

    void Initialize();

    bool docked() { return docked_; };

};
} // namespace iqr

#endif // DOCK_SERVER_H_