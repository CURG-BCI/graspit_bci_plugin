#ifndef STOPPEDGOHOMESTATE_H
#define STOPPEDGOHOMESTATE_H


#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/stoppedExecutionView.h"


class StoppedGoHomeState:public State
{
        Q_OBJECT
public:
    StoppedGoHomeState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, QState* parent = 0 );
     virtual void onEntryImpl(QEvent *e);
     virtual void onExitImpl(QEvent *e);

public slots:
    void onContinueExecutionClicked(){emit goToExecuteGoHomeState();};
    void onGoBackClicked(){emit goToBookmarkState();};

signals:
    void goToExecuteGoHomeState();
    void goToBookmarkState();


private:
    BCIControlWindow *bciControlWindow;
    StoppedExecutionView *stoppedExecutionView;
    ControllerSceneManager *csm;
};

#endif // STOPPEDGOHOMESTATE_H
