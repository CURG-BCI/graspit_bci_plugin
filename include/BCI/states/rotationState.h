#ifndef ROTATIONSTATE_H
#define ROTATIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/ManualAction.h"

class RotationState: public State
{
   Q_OBJECT

public:
    RotationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void positiveR();
    void negativeR();
    void positiveP();
    void negativeP();
    void positiveY();
    void negativeY();
    void onGoBack();

signals:
    void goToManualState();
    void goToExecuteRotationState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    actionlib::SimpleActionClient<graspit_msgs::ManualAction> rotationActionClient;


    void executeRotation(int32_t axis, int32_t direction);

    void executeRotationCallback(const actionlib::SimpleClientGoalState& state,
                                   const graspit_msgs::ManualResultConstPtr& result);
};


#endif
