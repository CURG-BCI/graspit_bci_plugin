#ifndef STOPPEDEXECUTIONSTATE_H
#define STOPPEDEXECUTIONSTATE_H


#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/stoppedExecutionView.h"


class StoppedExecutionState:public State
{
        Q_OBJECT
public:
    StoppedExecutionState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent = 0 );
     virtual void onEntryImpl(QEvent *e);
     virtual void onExitImpl(QEvent *e);
    void alexaCB(const std_msgs::String::ConstPtr& msg);

public slots:
    void onContinueExecutionClicked(){emit goToExecutionState();};
    void onStartOverClicked(){emit goToObjectSelectionState();};

signals:
    void goToExecutionState();
    void goToObjectSelectionState();


private:
    BCIControlWindow *bciControlWindow;
    StoppedExecutionView *stoppedExecutionView;
    ControllerSceneManager *csm;
    ros::Subscriber alexaSub;
    ros::Publisher alexaPub;
};

#endif // STOPPEDEXECUTIONSTATE_H
