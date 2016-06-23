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

    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                      -1.4, -0.6, 0.0, QString("Move\nArm")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -0.8, 0.0, QString("Open or\nClose Hand")));

    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -1.0, 0.0, QString("Back")));


    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToTranslationState()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(toggleGripper()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(emit_goToHomeState()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);

}


void ManualState::emit_goToTranslationState()
{
    emit goToTranslationState();
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



