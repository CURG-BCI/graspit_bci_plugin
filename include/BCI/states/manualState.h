#ifndef MANUALSTATE_H
#define MANUALSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/ToggleGripperAction.h"

class ManualState: public State
{
   Q_OBJECT

public:
    ManualState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void emit_goToTranslationState();
    void emit_goToRotationState();
    void toggleGripper();
    void emit_goToHomeState();

signals:
    void goToTranslationState();
    void goToRotationState();
    void goToHomeState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;

    actionlib::SimpleActionClient<graspit_msgs::ToggleGripperAction> toggleGripperActionClient;


    void toggleGripperCallback(const actionlib::SimpleClientGoalState& state,
                                   const graspit_msgs::ToggleGripperResultConstPtr& result);

};


#endif
