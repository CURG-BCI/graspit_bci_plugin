#include "BCI/states/graspSelectionState.h"
#include "BCI/bciService.h"
#include "BCI/graspManager.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include<Inventor/nodes/SoAnnotation.h>
#include "BCI/controller_scene/sprites.h"
#include <QPushButton>

using bci_experiment::GraspManager;

GraspSelectionState::GraspSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent):
    State("GraspSelectionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    stateName(QString("Grasp Selection"))
{
    addSelfTransition(BCIService::getInstance(),SIGNAL(plannerUpdated()), this, SLOT(render()));
    addSelfTransition(GraspManager::getInstance(),SIGNAL(render()), this, SLOT(render()));

    graspSelectionView = new GraspSelectionView(bciControlWindow->currentFrame);
    graspSelectionView->hide();

}

void GraspSelectionState::onEntryImpl(QEvent *e)
{

    WorldController::getInstance()->unhighlightAllBodies();
    GraspManager::getInstance()->startGraspReachabilityAnalysis();
    GraspManager::getInstance()->showRobots(true);

    graspSelectionView->show();
    bciControlWindow->currentState->setText(stateName);
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_selection.png"), -0.7 , 0.7, 0.0);
    render();
    csm->clearTargets();

    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                      QString("target_active.png"),
                                                                      -1.4,
                                                                      -0.4,
                                                                      0.0, QString("Select\nGrasp")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                      QString("target_background.png"),
                                                                      -1.4,
                                                                      -0.6,
                                                                      0.0, QString("Next\nGrasp")));

        std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                           QString("target_background.png"),
                                                                           -1.4,
                                                                           -0.8,
                                                                           0.0, QString("Plan New\nGrasps")));

    std::shared_ptr<Target>  t4 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                      QString("target_background.png"),
                                                                      -1.4,
                                                                      -1.0,
                                                                      0.0, QString("Back")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToConfirmationState()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onNext()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(emit_goToGraspPlanningState()));
    QObject::connect(t4.get(), SIGNAL(hit()), this, SLOT(emit_goToObjectSelectionState()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);
    csm->addTarget(t4);

    //do this to ensure that the side views are reset.
    GraspManager::getInstance()->decrementGraspIndex();
    onNext();

}


void GraspSelectionState::onExitImpl(QEvent *e)
{
    csm->clearTargets();
    SoDB::writelock();
    csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
    SoDB::writeunlock();
    delete csm->pipeline;
    csm->next_target=0;
    graspSelectionView->hide();

}



void GraspSelectionState::_updateCurrentGraspView()
{
    const GraspPlanningState * currentGrasp = GraspManager::getInstance()->getCurrentGrasp();

    if(currentGrasp)
    {
        Hand *graspDemoHand = GraspManager::getInstance()->getHand();
        graspSelectionView->showSelectedGrasp(graspDemoHand, currentGrasp);
        QString graspID;
        bciControlWindow->currentState->setText(stateName +"- Grasp: " + graspID.setNum(currentGrasp->getAttribute("graspId")) );
    }
}

void GraspSelectionState::_updateNextGraspView()
{
    const GraspPlanningState *nextGrasp = GraspManager::getInstance()->getNextGrasp();
    if(nextGrasp)
    {
        Hand *graspDemoHand = GraspManager::getInstance()->getHand();
        graspSelectionView->showNextGrasp(graspDemoHand, nextGrasp);
    }
}

void GraspSelectionState::showCurrentGrasp()
{
    const GraspPlanningState * currentGrasp = GraspManager::getInstance()->getCurrentGrasp();
    if (currentGrasp)
    {
        currentGrasp->execute(GraspManager::getInstance()->getHand());
    }
}

void GraspSelectionState::onNext()
{
    GraspManager::getInstance()->incrementGraspIndex();

    _updateCurrentGraspView();
    _updateNextGraspView();
    showCurrentGrasp();

    csm->setCursorPosition(-1,0,0);
}

void GraspSelectionState::render()
{
    ROS_INFO("GraspSelectionState::onPlannerUpdated()");
    static QTime activeTimer;
    qint64 minElapsedMSecs = 300;
    if(!activeTimer.isValid() || activeTimer.elapsed() >= minElapsedMSecs)
    {
        GraspManager::getInstance()->resetGraspIndex();

        _updateCurrentGraspView();
        _updateNextGraspView();
        showCurrentGrasp();
    }

    GraspManager::getInstance()->renderPending = false;
}



