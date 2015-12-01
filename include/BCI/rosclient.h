#ifndef ROSCLIENT_H
#define ROSCLIENT_H

#include <QObject>
#include "graspit_msgs/GetObjectInfo.h"
#include "graspit_msgs/ObjectInfo.h"
#include "moveit_trajectory_planner/LocationInfo.h"
#include "moveit_trajectory_planner/RunObjectRecognitionAction.h"
#include "moveit_trajectory_planner/CheckGraspReachabilityAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"

#include "include/matvec3D.h"

class GraspPlanningState;

class RosClient:public QObject
{

    Q_OBJECT

public:
    static RosClient* getInstance();

    void sendCheckGraspReachabilityRequest(const GraspPlanningState * state);

signals:
    void updateGraspReachability(int graspId, bool isReachable);

private:
    //singleton pattern, single static instance of the class
    static RosClient * rosClient;

    //this is singleton, so constructor must be private.
    RosClient();

    ros::NodeHandle *n;

    actionlib::SimpleActionClient<moveit_trajectory_planner::CheckGraspReachabilityAction> analzeGraspReachabilityActionClient;

    void buildCheckReachabilityRequest(const GraspPlanningState * gps, moveit_trajectory_planner::CheckGraspReachabilityGoal &goal);
    void checkGraspReachabilityCallback(const actionlib::SimpleClientGoalState& state,  const moveit_trajectory_planner::CheckGraspReachabilityResultConstPtr& result);
};

#endif // ROSCLIENT_H
