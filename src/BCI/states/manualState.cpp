#include "BCI/states/manualState.h"

#include "BCI/states/executionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "include/EGPlanner/searchState.h"
#include "graspit_msgs/ToggleGripperGoal.h"
#include <Inventor/nodes/SoAnnotation.h>


ManualState::ManualState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("ManualState", parent), bciControlWindow(_bciControlWindow),
      csm(_csm),
      toggleGripperActionClient("toggle_gripper_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();
}


void ManualState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Home State");

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), btn_x-1.5*btn_width, btn_y, 0.0, QString("Move\nArm"), this, SLOT(emit_goToTranslationState()));
    csm->addNewTarget(QString("target_background.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Rotate\nHand"), this, SLOT(emit_goToRotationState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Open or\nClose Hand"), this, SLOT(toggleGripper()));
    csm->addNewTarget(QString("target_background.png"), btn_x+1.5*btn_width, btn_y, 0.0, QString("Back"), this, SLOT(emit_goToHomeState()));

}


void ManualState::emit_goToTranslationState()
{
    emit goToTranslationState();
}

void ManualState::emit_goToRotationState()
{
    emit goToRotationState();
}

void ManualState::toggleGripper()
{
    graspit_msgs::ToggleGripperGoal goal;
    goal.toggle = true;
    toggleGripperActionClient.sendGoal(goal, boost::bind(&ManualState::toggleGripperCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::ToggleGripperAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::ToggleGripperAction>::SimpleFeedbackCallback());
}

void ManualState::emit_goToHomeState()
{
    emit goToHomeState();
}

void ManualState::toggleGripperCallback(const actionlib::SimpleClientGoalState& state,
                                            const graspit_msgs::ToggleGripperResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    if (!result->success)
    {
        std::cout << "Could not execute" << std::endl;
    }
}

void ManualState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Manual State." << std::endl;
}



