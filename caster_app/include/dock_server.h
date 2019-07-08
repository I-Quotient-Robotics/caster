#ifndef DOCK_SERVER_H_
#define DOCK_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/action_server.h>

#include <caster_app/DockAction.h>

namespace iqr {
class DockServer : public actionlib::ActionServer<caster_app::DockAction> {
  private:
    std::string action_name_;

    ros::NodeHandle private_nh_;

    GoalHandle goal_;

    caster_app::DockResult result_;
    caster_app::DockFeedback feedback_;

    void GoalCallback(GoalHandle gh);
    void CancelCallback(GoalHandle gh);

  public:

    typedef actionlib::ServerGoalHandle<caster_app::DockAction> GoalHandle;

    DockServer(ros::NodeHandle &nh, const std::string & server_name);

    void Initialize();

};
} // namespace iqr

#endif // DOCK_SERVER_H_