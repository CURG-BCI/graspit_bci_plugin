#include "BCI/states/stoppedGoHomeState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <Inventor/nodes/SoAnnotation.h>

StoppedGoHomeState::StoppedGoHomeState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    :State("StoppedExecutionState", parent), bciControlWindow(_bciControlWindow), csm(_csm)
{
    stoppedExecutionView = new StoppedExecutionView(bciControlWindow->currentFrame);
    stoppedExecutionView->hide();
}


void StoppedGoHomeState::onEntryImpl(QEvent *e)
{
    stoppedExecutionView->show();
    bciControlWindow->currentState->setText("Stopped Go Home State");

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Continue"), this, SLOT(onContinueExecutionClicked()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Back"), this, SLOT(onGoBackClicked()));
}


void StoppedGoHomeState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    stoppedExecutionView->hide();

}
