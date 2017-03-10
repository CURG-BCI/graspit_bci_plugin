#ifndef HOMESTATE_H
#define HOMESTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"

class HomeState: public State
{
   Q_OBJECT

public:
    HomeState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void emit_goToObjectSelectionState();
    void emit_goToManualState();
    void emit_goToBookmarkState();
    void emit_goToObjectRecognitionState();

signals:
    void goToObjectSelectionState();
    void goToManualState();
    void goToBookmarkState();
    void goToObjectRecognitionState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;

};


#endif
