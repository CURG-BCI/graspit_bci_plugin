#ifndef REACHABILITYANALYZER_H
#define REACHABILITYANALYZER_H

#include <QObject>
#include "graspit_msgs/GetObjectInfo.h"
#include "graspit_msgs/ObjectInfo.h"
#include "moveit_trajectory_planner/LocationInfo.h"
#include "moveit_trajectory_planner/RunObjectRecognitionAction.h"
#include "moveit_trajectory_planner/CheckGraspReachabilityAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"

#include "include/EGPlanner/searchState.h"

#include "include/matvec3D.h"


class ReachabilityAnalyzer: public QObject
{
    Q_OBJECT

public:

    ReachabilityAnalyzer();
    void sendCheckGraspReachabilityRequest(const GraspPlanningState * state);
    void setIsRunning(bool _isRunning){is_running = _isRunning;}
    bool isRunning(){return is_running;}

signals:
    void updateGraspReachability(int graspId, bool isReachable);

private:

    ros::NodeHandle *n;
    bool is_running;

    actionlib::SimpleActionClient<moveit_trajectory_planner::CheckGraspReachabilityAction> analzeGraspReachabilityActionClient;

    void buildCheckReachabilityRequest(const GraspPlanningState * gps, moveit_trajectory_planner::CheckGraspReachabilityGoal &goal);
    void checkGraspReachabilityCallback(const actionlib::SimpleClientGoalState& state,  const moveit_trajectory_planner::CheckGraspReachabilityResultConstPtr& result);
};

#endif //REACHABILITYANALYZER_H
