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
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       -1.4, -0.8, 0.0, QString("Continue")));
    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -1.0, 0.0, QString("Back")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(onContinueGoToBinClicked()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onGoBackClicked()));

    csm->addTarget(t1);
    csm->addTarget(t2);
}


void StoppedGoToBinState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    stoppedExecutionView->hide();


}
