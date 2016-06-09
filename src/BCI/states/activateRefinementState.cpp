#include "BCI/states/activateRefinementState.h"
#include "BCI/bciService.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "BCI/utils/BCILogger.h"

using bci_experiment::GraspManager;
using bci_experiment::world_element_tools::getWorld;
using bci_experiment::WorldController;

ActivateRefinementState::ActivateRefinementState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent):
    HandRotationState("ActivateRefinementState",_bciControlWindow, _csm, parent),
    csm(_csm)
{
    addSelfTransition(GraspManager::getInstance(),SIGNAL(render()), this, SLOT(updateView()));

    activeRefinementView = new ActiveRefinementView(bciControlWindow->currentFrame);
    activeRefinementView->hide();
}



void ActivateRefinementState::onEntryImpl(QEvent *e)
{
    activeRefinementView->show();
    updateView();
    bciControlWindow->currentState->setText("Refinement State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_selection.png"), -0.7 , 0.7, 0.0);

    csm->clearTargets();

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       -1.4, -0.6, 0.0, QString("Rotate\nLat")));
    t2->active=true;
    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                        -1.4, -0.8, 0.0, QString("Rotate\nLong")));
    std::shared_ptr<Target>  t4 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                        -1.4, -1.0, 0.0, QString("Finished\nRefinement")));

    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onRotateHandLat()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(onRotateHandLong()));
    QObject::connect(t4.get(), SIGNAL(hit()), this, SLOT(emit_returnToGraspSelectionState()));

    csm->addTarget(t2);
    csm->addTarget(t3);
    csm->addTarget(t4);

}

void ActivateRefinementState::setTimerRunning()
{
}

void ActivateRefinementState::onExitImpl(QEvent *e)
{
    csm->clearTargets();
    SoDB::writelock();
     csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
     SoDB::writeunlock();
    delete csm->pipeline;
     csm->next_target=0;
    activeRefinementView->hide();


    std::cout << "Finished onExit of Object Selection State." << std::endl;
}


void ActivateRefinementState::emit_returnToGraspSelectionState()
{
    BCIService::getInstance()->emitGoToNextState1();
}


void ActivateRefinementState::updateView()
{
    const GraspPlanningState *bestGrasp = GraspManager::getInstance()->getGrasp(0);
    const GraspPlanningState *nextGrasp = GraspManager::getInstance()->getNextGrasp();
    Hand *hand = GraspManager::getInstance()->getHand();

    if(nextGrasp)
    {
        activeRefinementView->showNextGrasp(hand, nextGrasp);
    }

    if(bestGrasp)
    {
        activeRefinementView->showSelectedGrasp(hand,bestGrasp);
        QString graspID;
        bciControlWindow->currentState->setText("Refinement State - Grasp:" + graspID.setNum(bestGrasp->getAttribute("graspId")) );
    }
    GraspManager::getInstance()->renderPending = false;

}


