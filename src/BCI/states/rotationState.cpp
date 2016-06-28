#include "BCI/states/rotationState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/ManualGoal.h"

RotationState::RotationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("RotationState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      rotationActionClient("manual_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();


}

void RotationState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("RotationState");

    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("axes.png"), -1.0 , 0.1, 0.0);

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), -1.4, 0.2 , 0.0, QString("+x"), this, SLOT(negativeR()));
    csm->addNewTarget(QString("target_background.png"), -1.4, 0.0 , 0.0, QString("-x"), this, SLOT(positiveR()));
    csm->addNewTarget(QString("target_background.png"), -1.4, -0.2 , 0.0, QString("+y"), this, SLOT(negativeP()));
    csm->addNewTarget(QString("target_background.png"), -1.4, -0.4 , 0.0, QString("-y"), this, SLOT(positiveP()));
    csm->addNewTarget(QString("target_background.png"), -1.4, -0.6 , 0.0, QString("+z"), this, SLOT(positiveY()));
    csm->addNewTarget(QString("target_background.png"), -1.4, -0.8 , 0.0, QString("-z"), this, SLOT(negativeY()));
    csm->addNewTarget(QString("target_background.png"), -1.4, -1.0 , 0.0, QString("Back"), this, SLOT(onGoBack()));

}

void RotationState::positiveR()
{
    executeRotation(3, 1);
}
void RotationState::negativeR()
{
    executeRotation(3, -1);
}
void RotationState::positiveP()
{
    executeRotation(4, 1);
}
void RotationState::negativeP()
{
    executeRotation(4, -1);
}
void RotationState::positiveY()
{
    executeRotation(5, 1);
}
void RotationState::negativeY()
{
    executeRotation(5, -1);
}

void RotationState::onGoBack()
{
    emit goToManualState();
}


void RotationState::executeRotation(int32_t axis, int32_t direction)
{
    // axis: roll = 3, pitch = 4, yaw = 5; direction: positive = 1, negative = -1
    graspit_msgs::ManualInfo message;
    message.axis = axis;
    message.direction = direction;
    graspit_msgs::ManualGoal goal;
    goal.manualinfo = message;
    rotationActionClient.sendGoal(goal, boost::bind(&RotationState::executeRotationCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::ManualAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::ManualAction>::SimpleFeedbackCallback());
    emit goToExecuteRotationState();


}
void RotationState::executeRotationCallback(const actionlib::SimpleClientGoalState& state,
                                            const graspit_msgs::ManualResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    if (!result->success)
    {
        std::cout << "Could not move in this direction" << std::endl;
    }
}

void RotationState::onExitImpl(QEvent *e)
{
    csm->clearTargets();
    SoDB::writelock();
    csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
    SoDB::writeunlock();
    delete csm->pipeline;


    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Translation State." << std::endl;


}
