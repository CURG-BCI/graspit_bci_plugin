
#include "BCI/states/confirmationState.h"
#include  <QSignalTransition>
#include <Inventor/nodes/SoAnnotation.h>
#include "BCI/onlinePlannerController.h"
#include "BCI/bciService.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"

#include <memory>

using bci_experiment::world_element_tools::getWorld;
using bci_experiment::OnlinePlannerController;

ConfirmationState::ConfirmationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent):
        State("ConfirmationState", parent),bciControlWindow(_bciControlWindow),
        csm(_csm)
{
    confirmationView = new ConfirmationView(bciControlWindow->currentFrame);
    this->addSelfTransition(BCIService::getInstance(),SIGNAL(rotLat()), this, SLOT(onNextGrasp()));
    confirmationView->hide();
}


void ConfirmationState::onEntryImpl(QEvent *e)
{
    const GraspPlanningState *grasp = OnlinePlannerController::getInstance()->getCurrentGrasp();
    Hand * hand = OnlinePlannerController::getInstance()->getSolutionHand();
    OnlinePlannerController::getInstance()->destroyGuides();
    confirmationView->setCurrentGrasp(hand,grasp);
    confirmationView->show();
    bciControlWindow->currentState->setText("Confirmation");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_confirmation.png"), -0.7 , 0.7, 0.0);
    OnlinePlannerController::getInstance()->setPlannerToPaused();


    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       -1.4, -0.8, 0.0, QString("Confirm\nGrasp")));
    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -1.0, 0.0, QString("Back")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToExecutionState()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(emit_goToPreviousState()));

    csm->addTarget(t1);
    csm->addTarget(t2);

}

void ConfirmationState::onNextGrasp(QEvent *e)
{

}

void ConfirmationState::onExitImpl(QEvent * e)
{
    csm->clearTargets();
    SoDB::writelock();
     csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
     SoDB::writeunlock();
     csm->next_target=0;
    delete csm->pipeline;
    Q_UNUSED(e);
    confirmationView->hide();


    std::cout << "Finished onExit of Object Selection State." << std::endl;

}


