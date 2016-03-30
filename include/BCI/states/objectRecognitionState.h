#ifndef OBJECTRECOGNITIONSTATE_H
#define OBJECTRECOGNITIONSTATE_H

#include "state_views/objectRecognitionView.h"
#include "bciControlWindow.h"
#include "BCI/state.h"

#include "graspit_msgs/GetObjectInfo.h"
#include "graspit_msgs/ObjectInfo.h"
#include "graspit_msgs/LocationInfo.h"
#include "graspit_msgs/RunObjectRecognitionAction.h"
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
   actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction> recognizeObjectsActionClient;
   ros::ServiceClient get_camera_origin;
   bool use_hardware;
   bool rec_running;

   void sendObjectRecognitionRequest();
   void sendGetCameraOriginRequest();
   void objectRecognitionCallback(const actionlib::SimpleClientGoalState& state,
                                  const graspit_msgs::RunObjectRecognitionResultConstPtr& result);
   void addObject(graspit_msgs::ObjectInfo object);


};

#endif // OBJECTRECOGNITIONSTATE_H
