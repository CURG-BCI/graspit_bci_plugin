#ifndef GRASPSELECTION_STATE_H
#define GRASPSELECTION_STATE_H

#ifndef Q_MOC_RUN
#include "state_views/graspPlanningView.h"
#include "bciControlWindow.h"
#include "BCI/state.h"

#include "BCIOnlinePlanner.h"

#include "graspit_msgs/GetObjectInfo.h"
#include "graspit_msgs/ObjectInfo.h"
#include "graspit_msgs/LocationInfo.h"
#include "graspit_msgs/RunObjectRecognitionAction.h"
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#endif

class BCIListPlanner;

class PlanGraspState : public State
{
        Q_OBJECT
public:
    PlanGraspState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);


public slots:
    void emit_goToGraspSelectionState() {emit goToGraspSelectionState();}
    void onPlannerFinished();

signals:
    void goToGraspSelectionState();

private:
   GraspPlanningView *graspPlanningView;
   BCIControlWindow *bciControlWindow;
   ControllerSceneManager *csm;

   BCIListPlanner *mPlanner;
   GraspPlanningState *mHandObjectState;
   GraspableBody *mObject;
   Hand *mHand;

   std::string experiment_type;
   double fingerLength;

    ros::Publisher alexaPub;
    ros::NodeHandle *n;

   void cylinderSampling(double a, double b, double c, int resLen, int resRot);
   void smallSphereSampling(double a, double b, double c, int resLen, int resRot);
   void smallCubeSampling(double c, int res, double offset);
   void blockSampling(double a, double b, double c, int resLen, int resRot);
   void addNewGrasp(transf tr, std::list<GraspPlanningState*> *sampling);
};

#endif // GRASPSELECTION_STATE_H
