#ifndef OBJECTSELECTIONSTATE_H
#define OBJECTSELECTIONSTATE_H

#include <QObject>
#include <QSignalTransition>
#include "BCI/state.h"
#include "BCI/graspManager.h"
#include "bciControlWindow.h"

class ObjectSelectionView;

class ObjectSelectionState:public State
{
    Q_OBJECT

public:
    ObjectSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent = 0 );
    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

private:
    BCIControlWindow *bciControlWindow;
    ObjectSelectionView *objectSelectionView;
    BCIService *bciService;
    ControllerSceneManager *csm;


public slots:

    void onNext();
    void onSelect();
    void onGoBack();


signals:
    void goToNextState();

};

#endif // OBJECTSELECTIONSTATE_H
