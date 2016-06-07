#ifndef GRASPSELECTIONSTATE_H
#define GRASPSELECTIONSTATE_H


#include "BCI/states/handRotationState.h"

#include <QSignalTransition>
#include "include/debug.h"
#include "bciControlWindow.h"
#include "state_views/graspSelectionView.h"

class GraspSelectionState: public State
{
    Q_OBJECT

public:
    GraspSelectionState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, QState* parent = 0 );
    QString stateName;
    BCIControlWindow *bciControlWindow;

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

protected:
    GraspSelectionView *graspSelectionView;
    ControllerSceneManager *csm;

    void _updateCurrentGraspView();
    void _updateNextGraspView();
    void showCurrentGrasp();

public slots:
    void onNext();
    void onPlannerUpdated();


    void emit_goToActivateRefinementState(){emit goToActivateRefinementState();}
    void emit_goToObjectSelectionState(){emit goToObjectSelectionState();}
    void emit_goToConfirmationState(){emit goToConfirmationState();}


signals:
    void goToActivateRefinementState();
    void goToObjectSelectionState();
    void goToConfirmationState();



};


#endif // GRASPSELECTIONSTATE_H
