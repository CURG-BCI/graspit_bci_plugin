#ifndef ActivateRefinementState_H
#define ActivateRefinementState_H

#include "BCI/states/handRotationState.h"

#include <QSignalTransition>
#include "include/debug.h"
#include "bciControlWindow.h"
#include "state_views/activeRefinementView.h"

class ActivateRefinementState: public HandRotationState
{
    Q_OBJECT

public:
    ActivateRefinementState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent = 0 );


public slots:
    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);
    virtual void updateView();

    void emit_returnToGraspSelectionState();

    void setTimerRunning();

protected:
    ActiveRefinementView *activeRefinementView;

private:
    ControllerSceneManager *csm;


};


#endif // ActivateRefinementState_H


