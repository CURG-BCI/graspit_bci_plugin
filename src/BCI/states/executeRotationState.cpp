#include "BCI/states/executeRotationState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/ManualGoal.h"

ExecuteRotationState::ExecuteRotationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("ExecuteRotationState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      rotationActionClient("manual_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();


}

void ExecuteRotationState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Execute Rotation State");

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), -1.4, -1.0 , 0.0, QString("STOP!"), this, SLOT(onStop()));


}

void ExecuteRotationState::onStop()
{
    rotationActionClient.cancelAllGoals();
    emit goToRotationState();
}


void ExecuteRotationState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Translation State." << std::endl;
}
