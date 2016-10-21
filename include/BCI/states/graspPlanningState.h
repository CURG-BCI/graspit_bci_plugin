#ifndef GRASPSELECTION_STATE_H
#define GRASPSELECTION_STATE_H

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
class ListPlanner;

class PlanGraspState : public State
{
        Q_OBJECT
public:
    PlanGraspState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

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

//   EGPlanner *mPlanner;
   ListPlanner *mPlanner;
   GraspPlanningState *mHandObjectState;
   GraspableBody *mObject;
   Hand *mHand;
   void gridEllipsoidSampling(const GraspPlanningState &seed,
                                              std::list<GraspPlanningState*> *sampling,
                                              int samples);
   void ellipsoidSampling(double a, double b, double c, double resolution);
   void addCartesianSamples(const GraspPlanningState &seed,
                                            std::list<GraspPlanningState*> *sampling,
                                            int samples, double x, double y, double z);
   void sampleFace(vec3 x, vec3 y, vec3 z, transf ref_tran,
                                   double sz1, double sz2, vec3 tln, double res,
                                   std::list<GraspPlanningState*> *sampling);
   void boxSampling(double a, double b, double c, double res);
   void cylinderSampling(double a, double b, double c, double resolution);
   void sampleCylinderSlices(transf seed_ref_tran, int axis,
                                   double length, double radius, double res,
                                   std::list<GraspPlanningState*> *sampling);
};

#endif // GRASPSELECTION_STATE_H
