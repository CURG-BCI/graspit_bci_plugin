#ifndef STOPPEDEXECUTIONSTATE_H
#define STOPPEDEXECUTIONSTATE_H


#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/stoppedExecutionView.h"


class StoppedExecutionState:public State
{
        Q_OBJECT
public:
    StoppedExecutionState(BCIControlWindow *_bciControlWindow, QState* parent = 0 );
     virtual void onEntry(QEvent *e);
     virtual void onExit(QEvent *e);

private:
    BCIControlWindow *bciControlWindow;
    StoppedExecutionView *stoppedExecutionView;

};

#endif // STOPPEDEXECUTIONSTATE_H
