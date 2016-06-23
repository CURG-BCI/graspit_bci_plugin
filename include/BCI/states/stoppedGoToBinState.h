#ifndef STOPPEDGOTOBINSTATE_H
#define STOPPEDGOTOBINSTATE_H


#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/stoppedExecutionView.h"


class StoppedGoToBinState:public State
{
        Q_OBJECT
public:
    StoppedGoToBinState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, QState* parent = 0 );
     virtual void onEntryImpl(QEvent *e);
     virtual void onExitImpl(QEvent *e);

public slots:
    void onContinueGoToBinClicked(){emit goToExecuteGoToBinState();};
    void onGoBackClicked(){emit goToBookmarkState();};

signals:
    void goToExecuteGoToBinState();
    void goToBookmarkState();


private:
    BCIControlWindow *bciControlWindow;
    StoppedExecutionView *stoppedExecutionView;
    ControllerSceneManager *csm;
};

#endif // STOPPEDGOTOBINSTATE_H
