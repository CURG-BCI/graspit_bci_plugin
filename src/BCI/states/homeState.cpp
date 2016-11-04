#include "BCI/states/homeState.h"

#include "BCI/states/executionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>

using bci_experiment::GraspManager;

HomeState::HomeState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("HomeState", parent), bciControlWindow(_bciControlWindow),
      csm(_csm)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();
}

void HomeState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Home State");

    GraspManager::getInstance()->showRobots(false);

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), btn_x-1.5*btn_width, btn_y, 0.0, QString("Auto\nGrasp"), this, SLOT(emit_goToObjectSelectionState()));
    csm->addNewTarget(QString("target_background.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Manual\nMovement"), this, SLOT(emit_goToManualState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Bookmarks"), this, SLOT(emit_goToBookmarkState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+1.5*btn_width, btn_y, 0.0, QString("Rerun\nVision"), this, SLOT(emit_goToObjectRecognitionState()));
}

void HomeState::emit_goToObjectSelectionState()
{
    emit goToObjectSelectionState();
}

void HomeState::emit_goToManualState()
{
    emit goToManualState();
}

void HomeState::emit_goToBookmarkState()
{
    emit goToBookmarkState();
}

void HomeState::emit_goToObjectRecognitionState()
{
    emit goToObjectRecognitionState();
}

void HomeState::onExitImpl(QEvent *e)
{
    std::cout << "Starting onExit of Home State." << std::endl;
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Home State." << std::endl;
}



