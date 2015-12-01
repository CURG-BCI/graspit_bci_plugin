#ifndef STARTSTATE_H
#define STARTSTATE_H

#include "BCI/states/handRotationState.h"

#include <QSignalTransition>
#include "include/debug.h"
#include "bciControlWindow.h"
#include "state_views/activeRefinementView.h"

class ActivateRefinementState: public HandRotationState
{

public:
    ActivateRefinementState(BCIControlWindow *_bciControlWindow, QState* parent = 0 );

public slots:
    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);
    virtual void onPlannerUpdated(QEvent *e = NULL);

protected:
    ActiveRefinementView *activeRefinementView;


};


#endif // STARTSTATE_H


