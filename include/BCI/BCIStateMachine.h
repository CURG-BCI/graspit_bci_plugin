#ifndef BCISTATEMACHINE_H
#define BCISTATEMACHINE_H

#include <QState>
#include <QStateMachine>
#include <QSignalTransition>

#include "state.h"
#include "ui/bciControlWindow.h"


#include "include/ivmgr.h"


class BCIStateMachine
{
public:
    BCIStateMachine(BCIControlWindow *_bciControlWindow, BCIService *_bciService);
    void start();


private:
    BCIControlWindow *bciControlWindow;
    BCIService *bciService;
    QStateMachine stateMachine;

};

#endif // BCISTATEMACHINE_H
