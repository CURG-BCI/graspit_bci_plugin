#include "BCI/states/translationState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/ManualGoal.h"

TranslationState::TranslationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("TranslationState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      translationActionClient("manual_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();


}

void TranslationState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Translation State");

    csm->clearTargets();

//    csm->addNewTarget(QString("target_active.png"), -1.4, 0.2 , 0.0, QString("Up"), this, SLOT(positiveZ()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, 0.0 , 0.0, QString("Down"), this, SLOT(negativeZ()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, -0.2 , 0.0, QString("Right"), this, SLOT(negativeX()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, -0.4 , 0.0, QString("Left"), this, SLOT(positiveX()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, -0.6 , 0.0, QString("Towards"), this, SLOT(positiveY()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, -0.8 , 0.0, QString("Away"), this, SLOT(negativeY()));
//    csm->addNewTarget(QString("target_background.png"), -1.4, -1.0 , 0.0, QString("Back"), this, SLOT(onGoBack()));

    csm->addNewTarget(QString("zPosActive.PNG"), -1.4, 0.2 , 0.0, QString(""), this, SLOT(positiveZ()),"zPosBackground.PNG", "zPosActive.PNG");
    csm->addNewTarget(QString("zNegBackground.PNG"), -1.4, 0.0 , 0.0, QString(""), this, SLOT(negativeZ()), "zNegBackground.PNG", "zNegActive.PNG");
    csm->addNewTarget(QString("yPosBackground.PNG"), -1.4, -0.2 , 0.0, QString(""), this, SLOT(negativeX()), "yPosBackground.PNG", "yPosActive.PNG");
    csm->addNewTarget(QString("yNegBackground.PNG"), -1.4, -0.4 , 0.0, QString(""), this, SLOT(positiveX()), "yNegBackground.PNG", "yNegActive.PNG");
    csm->addNewTarget(QString("xPosBackground.PNG"), -1.4, -0.6 , 0.0, QString(""), this, SLOT(positiveY()), "xPosBackground.PNG", "xPosActive.PNG");
    csm->addNewTarget(QString("xNegBackground.PNG"), -1.4, -0.8 , 0.0, QString(""), this, SLOT(negativeY()), "xNegBackground.PNG", "xNegActive.PNG");
    csm->addNewTarget(QString("target_background.png"), -1.4, -1.0 , 0.0, QString("Back"), this, SLOT(onGoBack()));

}

void TranslationState::positiveX()
{
    executeTranslation(0, 1);
}
void TranslationState::negativeX()
{
    executeTranslation(0, -1);
}
void TranslationState::positiveY()
{
    executeTranslation(1, 1);
}
void TranslationState::negativeY()
{
    executeTranslation(1, -1);
}
void TranslationState::positiveZ()
{
    executeTranslation(2, 1);
}
void TranslationState::negativeZ()
{
    executeTranslation(2, -1);
}

void TranslationState::onGoBack()
{
    emit goToManualState();
}


void TranslationState::executeTranslation(int32_t axis, int32_t direction)
{
    // axis: x = 0, y = 1, z = 2; direction: positive = 1, negative = -1
    graspit_msgs::ManualInfo message;
    message.axis = axis;
    message.direction = direction;
    graspit_msgs::ManualGoal goal;
    goal.manualinfo = message;
    translationActionClient.sendGoal(goal, boost::bind(&TranslationState::executeTranslationCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::ManualAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::ManualAction>::SimpleFeedbackCallback());
    emit goToExecuteTranslationState();


}
void TranslationState::executeTranslationCallback(const actionlib::SimpleClientGoalState& state,
                                            const graspit_msgs::ManualResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    if (!result->success)
    {
        std::cout << "Could not move in this direction" << std::endl;
    }
}

void TranslationState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Translation State." << std::endl;
}
