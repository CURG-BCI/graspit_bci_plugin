#ifndef EXECUTIONSTATE_H
#define EXECUTIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"

class ExecutionState: public State
{
        Q_OBJECT
public:
    ExecutionState(BCIControlWindow *_bciControlWindow, QState* parent=0);

    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
};

#endif // EXECUTIONSTATE_H
