#ifndef EXECUTIONSTATE_H
#define EXECUTIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/GraspExecutionAction.h"
#include <actionlib/client/simple_action_client.h>

class GraspPlanningState;

class ExecutionState: public State
{
   Q_OBJECT

public:
    ExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);
    void alexaCB(const std_msgs::String::ConstPtr& msg);

public slots:
    void emit_goToStoppedExecutionState();
    void emit_goToHomeState();

signals:
    void goToStoppedExecutionState();
    void goToHomeState();
    void goToBookmarkState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    ros::Publisher grasp_execution_pubisher;
    ros::Publisher grasp_stop_execution_pubisher;
    ros::Publisher grasp_stop_fingers_execution_pubisher;
    ros::ServiceClient grasp_serviceclient;
    actionlib::SimpleActionClient<graspit_msgs::GraspExecutionAction> graspExecutionActionClient;
    ros::Subscriber alexaSub;

    void executeGrasp(const GraspPlanningState * gps);

    void graspExecutionCallback(const actionlib::SimpleClientGoalState& state,
                                   const graspit_msgs::GraspExecutionResultConstPtr& result);
};


#endif
