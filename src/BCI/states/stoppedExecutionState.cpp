#include "BCI/states/stoppedExecutionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"

StoppedExecutionState::StoppedExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    :State("StoppedExecutionState", parent), bciControlWindow(_bciControlWindow), csm(_csm)
{
    stoppedExecutionView = new StoppedExecutionView(bciControlWindow->currentFrame);
    stoppedExecutionView->hide();
}


void StoppedExecutionState::onEntry(QEvent *e)
{
    stoppedExecutionView->show();
    bciControlWindow->currentState->setText("Execution");

    csm->clearTargets();
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_paused_execution.png"), -0.7 , 0.7, 0.0);
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -1.0, 0.0, QString("Continue")));
    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -0.8, 0.0, QString("Start\nOver")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(onContinueExecutionClicked()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onStartOverClicked()));

    csm->addTarget(t1);
    csm->addTarget(t2);
}


void StoppedExecutionState::onExit(QEvent *e)
{   delete csm->pipeline;
    stoppedExecutionView->hide();
}
