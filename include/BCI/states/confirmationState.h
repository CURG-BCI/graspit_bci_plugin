#ifndef CONFIRMATIONSTATE_H
#define CONFIRMATIONSTATE_H


#include "bciControlWindow.h"
#include "BCI/state.h"
#include "state_views/confirmationView.h"
class ConfirmationState:public State
{
    Q_OBJECT

public:
    ConfirmationState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent = 0 );

     virtual void onEntryImpl(QEvent *e);
     virtual void onExitImpl(QEvent *e);


public slots:
    void onNextGrasp(QEvent *e=NULL);
private:
    ConfirmationView* confirmationView;
    BCIControlWindow *bciControlWindow;
    ControllerSceneManager *csm;

public slots:

    void emit_goToExecutionState(){emit goToExecutionState();}
    void emit_goToPreviousState(){emit goToPreviousState();}

signals:
    void goToExecutionState();
    void goToPreviousState();
};


#endif // CONFIRMATIONSTATE_H
