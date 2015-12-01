#ifndef OBJECTRECOGNITIONSTATE_H
#define OBJECTRECOGNITIONSTATE_H

#include "state_views/objectRecognitionView.h"
#include "bciControlWindow.h"
#include "BCI/state.h"

#include "graspit_msgs/GetObjectInfo.h"
#include "graspit_msgs/ObjectInfo.h"
#include "moveit_trajectory_planner/LocationInfo.h"
#include "moveit_trajectory_planner/RunObjectRecognitionAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"

class ObjectRecognitionState : public State
{
        Q_OBJECT
public:
    ObjectRecognitionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);

signals:
    void addToWorld(const QString model_filename, const QString object_name, const transf object_pose);
    void clearGB();

private:
   ObjectRecognitionView *objectRecognitionView;
   BCIControlWindow *bciControlWindow;
   ControllerSceneManager *csm;
   actionlib::SimpleActionClient<moveit_trajectory_planner::RunObjectRecognitionAction> recognizeObjectsActionClient;
   ros::ServiceClient get_camera_origin;

   void sendObjectRecognitionRequest();
   void sendGetCameraOriginRequest();
   void objectRecognitionCallback(const actionlib::SimpleClientGoalState& state,
                                  const moveit_trajectory_planner::RunObjectRecognitionResultConstPtr& result);
   void addObject(graspit_msgs::ObjectInfo object);


};

#endif // OBJECTRECOGNITIONSTATE_H
