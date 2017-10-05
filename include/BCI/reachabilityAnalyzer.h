#ifndef REACHABILITYANALYZER_H
#define REACHABILITYANALYZER_H

#include <QObject>

#ifndef Q_MOC_RUN
#include "graspit_msgs/CheckGraspReachabilityAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#endif

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

    actionlib::SimpleActionClient<graspit_msgs::CheckGraspReachabilityAction> analzeGraspReachabilityActionClient;

    void buildCheckReachabilityRequest(const GraspPlanningState * gps, graspit_msgs::CheckGraspReachabilityGoal &goal);
    void checkGraspReachabilityCallback(const actionlib::SimpleClientGoalState& state,  const graspit_msgs::CheckGraspReachabilityResultConstPtr& result);
};

#endif //REACHABILITYANALYZER_H
