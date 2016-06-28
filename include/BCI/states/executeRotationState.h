#ifndef EXECUTEROTATIONSTATE_H
#define EXECUTEROTATIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/ManualAction.h"

class ExecuteRotationState: public State
{
   Q_OBJECT

public:
    ExecuteRotationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void onStop();

signals:
    void goToRotationState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    actionlib::SimpleActionClient<graspit_msgs::ManualAction> rotationActionClient;



};


#endif
