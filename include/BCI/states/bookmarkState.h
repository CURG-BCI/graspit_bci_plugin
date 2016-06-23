#ifndef BOOKMARKSTATE_H
#define BOOKMARKSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"

class BookmarkState: public State
{
   Q_OBJECT

public:
    BookmarkState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void emit_goToExecuteGoHomeState();
    void emit_goToExecuteGoToBinState();
    void emit_goToHomeState();

signals:
    void goToExecuteGoHomeState();
    void goToExecuteGoToBinState();
    void goToHomeState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;

};


#endif
