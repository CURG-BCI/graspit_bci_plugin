#include "BCI/rosclient.h"

#include <Inventor/nodes/SoCamera.h>

#include "get_camera_origin/GetCameraOrigin.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include "graspit_msgs/AnalyzePose.h"
#include "graspit_msgs/Grasp.h"

#include "include/debug.h"
#include "include/graspitGUI.h"
#include "include/ivmgr.h"
#include "include/grasp.h"
#include "include/EGPlanner/searchState.h"

#include "BCI/onlinePlannerController.h"
#include "BCI/bciService.h"


using bci_experiment::OnlinePlannerController;
using namespace moveit_trajectory_planner;

RosClient * RosClient::rosClient = NULL;

RosClient* RosClient::getInstance()
{
    if(!rosClient)
    {
        rosClient = new RosClient();
    }

    return rosClient;
}

RosClient::RosClient():
      analzeGraspReachabilityActionClient("analyze_grasp_action", true)
{
    ROS_INFO("Ros Client starting to come up...");
    n = new ros::NodeHandle("");
    ROS_INFO("Ros Client Initialized");

}


void RosClient::buildCheckReachabilityRequest(const GraspPlanningState * gps, moveit_trajectory_planner::CheckGraspReachabilityGoal &goal)
{
    goal.grasp.object_name = gps->getObject()->getName().toStdString().c_str();
    goal.grasp.epsilon_quality=gps->getEpsilonQuality();
    goal.grasp.volume_quality=gps->getVolume();
    goal.grasp.grasp_id=gps->getAttribute("graspId");

    double dof[gps->getHand()->getNumDOF()];
    const_cast<GraspPlanningState *>(gps)->getPosture()->getHandDOF(dof);
    for(int i = 0; i < gps->getHand()->getNumDOF(); ++i)
    {
       goal.grasp.final_grasp_dof.push_back(dof[i]);
       goal.grasp.pre_grasp_dof.push_back(dof[i]);
    }

    transf finalHandTransform = gps->readPosition()->getCoreTran();

    float tx = finalHandTransform.translation().x() / 1000;
    float ty = finalHandTransform.translation().y() / 1000;
    float tz = finalHandTransform.translation().z() / 1000;
    float rw = finalHandTransform.rotation().w;
    float rx = finalHandTransform.rotation().x;
    float ry = finalHandTransform.rotation().y;
    float rz = finalHandTransform.rotation().z;

    goal.grasp.final_grasp_pose.position.x=tx ;
    goal.grasp.final_grasp_pose.position.y=ty;
    goal.grasp.final_grasp_pose.position.z=tz;
    goal.grasp.final_grasp_pose.orientation.w=rw;
    goal.grasp.final_grasp_pose.orientation.x=rx;
    goal.grasp.final_grasp_pose.orientation.y=ry;
    goal.grasp.final_grasp_pose.orientation.z=rz;

    double moveDist = -50.0;
    transf pregraspHandTransform = (translate_transf(vec3(0,0,moveDist) * gps->getHand()->getApproachTran()) * gps->readPosition()->getCoreTran());
    tx = pregraspHandTransform.translation().x()/ 1000;
    ty = pregraspHandTransform.translation().y()/ 1000;
    tz = pregraspHandTransform.translation().z()/ 1000;
    rw = pregraspHandTransform.rotation().w;
    rx = pregraspHandTransform.rotation().x;
    ry = pregraspHandTransform.rotation().y;
    rz = pregraspHandTransform.rotation().z;

    goal.grasp.pre_grasp_pose.position.x=tx;
    goal.grasp.pre_grasp_pose.position.y=ty;
    goal.grasp.pre_grasp_pose.position.z=tz;
    goal.grasp.pre_grasp_pose.orientation.w=rw;
    goal.grasp.pre_grasp_pose.orientation.x=rx;
    goal.grasp.pre_grasp_pose.orientation.y=ry;
    goal.grasp.pre_grasp_pose.orientation.z=rz;
}


void RosClient::sendCheckGraspReachabilityRequest(const GraspPlanningState * gps)
{

    CheckGraspReachabilityGoal goal;
    buildCheckReachabilityRequest(gps, goal);

    analzeGraspReachabilityActionClient.sendGoal(goal,  boost::bind(&RosClient::checkGraspReachabilityCallback, this, _1, _2),
                actionlib::SimpleActionClient<moveit_trajectory_planner::CheckGraspReachabilityAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<moveit_trajectory_planner::CheckGraspReachabilityAction>::SimpleFeedbackCallback());

}

void RosClient::checkGraspReachabilityCallback(const actionlib::SimpleClientGoalState& state,
                                               const moveit_trajectory_planner::CheckGraspReachabilityResultConstPtr& result)
{
    emit updateGraspReachability(result->grasp_id, result->isPossible);
}

