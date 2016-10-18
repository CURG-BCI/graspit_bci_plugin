#include "BCI/states/stoppedGoToBinState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <Inventor/nodes/SoAnnotation.h>

StoppedGoToBinState::StoppedGoToBinState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    :State("StoppedGoToBinState", parent), bciControlWindow(_bciControlWindow), csm(_csm)
{
    stoppedExecutionView = new StoppedExecutionView(bciControlWindow->currentFrame);
    stoppedExecutionView->hide();
}


void StoppedGoToBinState::onEntryImpl(QEvent *e)
{
    stoppedExecutionView->show();
    bciControlWindow->currentState->setText("Stopped Go To Bin State");

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Continue"), this, SLOT(onContinueGoToBinClicked()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Back"), this, SLOT(onGoBackClicked()));
}


void StoppedGoToBinState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    stoppedExecutionView->hide();


}
