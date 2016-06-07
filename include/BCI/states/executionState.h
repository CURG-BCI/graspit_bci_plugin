#ifndef EXECUTIONSTATE_H
#define EXECUTIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"

class GraspPlanningState;

class ExecutionState: public State
{
   Q_OBJECT

public:
    ExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void emit_goToStoppedExecutionState();

signals:
    void goToStoppedExecutionState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    ros::Publisher grasp_execution_pubisher;
    ros::Publisher grasp_stop_execution_pubisher;
    ros::Publisher grasp_stop_fingers_execution_pubisher;

    void executeGrasp(const GraspPlanningState * gps);
};


#endif
