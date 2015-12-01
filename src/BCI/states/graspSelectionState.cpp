#include "BCI/states/graspSelectionState.h"
#include "BCI/bciService.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <QPushButton>

using bci_experiment::OnlinePlannerController;

GraspSelectionState::GraspSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent):
    State("GraspSelectionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    stateName(QString("Grasp Selection"))
{
    addSelfTransition(OnlinePlannerController::getInstance(),SIGNAL(render()), this, SLOT(onPlannerUpdated()));

    graspSelectionView = new GraspSelectionView(bciControlWindow->currentFrame);
    graspSelectionView->hide();

}

void GraspSelectionState::onEntry(QEvent *e)
{
    graspSelectionView->show();
    bciControlWindow->currentState->setText(stateName);

    onPlannerUpdated();

    csm->clearTargets();

    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       0.35,
                                                                       0.25,
                                                                       0.0, QString("Next\nGrasp")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.1,
                                                                       0.25,
                                                                       0.0, QString("Refine\nGrasp")));
    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.1,
                                                                       -1.0,
                                                                       0.0, QString("Select\nGrasp")));

    std::shared_ptr<Target>  t4 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       0.35,
                                                                       -1.0,
                                                                       0.0, QString("Select\nDifferent\nObject")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(onNext()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(emit_goToActivateRefinementState()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(emit_goToConfirmationState()));
    QObject::connect(t4.get(), SIGNAL(hit()), this, SLOT(emit_goToObjectSelectionState()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);
    csm->addTarget(t4);
}


void GraspSelectionState::onExit(QEvent *e)
{
    csm->clearTargets();
    graspSelectionView->hide();
}



void GraspSelectionState::_updateCurrentGraspView()
{
    const GraspPlanningState * currentGrasp = OnlinePlannerController::getInstance()->getCurrentGrasp();
    if(currentGrasp)
    {
        Hand *graspDemoHand = OnlinePlannerController::getInstance()->getSolutionHand();
        graspSelectionView->showSelectedGrasp(graspDemoHand, currentGrasp);
        QString graspID;
        bciControlWindow->currentState->setText(stateName +"- Grasp: " + graspID.setNum(currentGrasp->getAttribute("graspId")) );
    }
}

void GraspSelectionState::_updateNextGraspView()
{
    const GraspPlanningState *nextGrasp = OnlinePlannerController::getInstance()->getNextGrasp();
    if(nextGrasp)
    {
        Hand *graspDemoHand = OnlinePlannerController::getInstance()->getSolutionHand();
        graspSelectionView->showNextGrasp(graspDemoHand, nextGrasp);
    }
}

void GraspSelectionState::showCurrentGrasp()
{
    const GraspPlanningState * currentGrasp = OnlinePlannerController::getInstance()->getCurrentGrasp();
    if (currentGrasp)
    {
        currentGrasp->execute(OnlinePlannerController::getInstance()->getSolutionHand());
        OnlinePlannerController::getInstance()->alignHand();
    }
}

void GraspSelectionState::onNext()
{
    OnlinePlannerController::getInstance()->incrementGraspIndex();

    _updateCurrentGraspView();
    _updateNextGraspView();
    showCurrentGrasp();

    csm->setCursorPosition(-1,0,0);
}

void GraspSelectionState::onPlannerUpdated()
{
    static QTime activeTimer;
    qint64 minElapsedMSecs = 300;
    if(!activeTimer.isValid() || activeTimer.elapsed() >= minElapsedMSecs)
    {
        OnlinePlannerController::getInstance()->sortGrasps();
        OnlinePlannerController::getInstance()->resetGraspIndex();

        _updateCurrentGraspView();
        _updateNextGraspView();
        showCurrentGrasp();
    }

    OnlinePlannerController::getInstance()->renderPending = false;
}



