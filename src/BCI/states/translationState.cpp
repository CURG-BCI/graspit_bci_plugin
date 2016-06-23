#include "BCI/states/translationState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/TranslationGoal.h"

TranslationState::TranslationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("TranslationState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      translationActionClient("translation_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();


}

void TranslationState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Translation State");

    csm->clearTargets();
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                      -1.0, -0.8, 0.0, QString("Right")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.2, -1.0, 0.0, QString("Down")));

    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -0.8, 0.0, QString("Left")));

    std::shared_ptr<Target>  t4 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.2, -0.6, 0.0, QString("Up")));

    std::shared_ptr<Target>  t5 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -0.8, -0.7, 0.0, QString("Towards")));

    std::shared_ptr<Target>  t6 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -0.8, -0.9, 0.0, QString("Away")));

    std::shared_ptr<Target>  t7 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -0.6, -0.8, 0.0, QString("Back")));


    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(negativeX()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(negativeZ()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(positiveX()));
    QObject::connect(t4.get(), SIGNAL(hit()), this, SLOT(positiveZ()));
    QObject::connect(t5.get(), SIGNAL(hit()), this, SLOT(positiveY()));
    QObject::connect(t6.get(), SIGNAL(hit()), this, SLOT(negativeY()));
    QObject::connect(t7.get(), SIGNAL(hit()), this, SLOT(onGoBack()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);
    csm->addTarget(t4);
    csm->addTarget(t5);
    csm->addTarget(t6);
    csm->addTarget(t7);

}

void TranslationState::positiveX()
{
    executeTranslation("x", "positive");
}
void TranslationState::negativeX()
{
    executeTranslation("x", "negative");
}
void TranslationState::positiveY()
{
    executeTranslation("y", "positive");
}
void TranslationState::negativeY()
{
    executeTranslation("y", "negative");
}
void TranslationState::positiveZ()
{
    executeTranslation("z", "positive");
}
void TranslationState::negativeZ()
{
    executeTranslation("z", "negative");
}

void TranslationState::onGoBack()
{
    emit goToManualState();
}


void TranslationState::executeTranslation(std::string axis, std::string direction)
{
    graspit_msgs::TranslationInfo message;
    message.axis = axis;
    message.direction = direction;
    graspit_msgs::TranslationGoal goal;
    goal.translationinfo = message;
    translationActionClient.sendGoal(goal, boost::bind(&TranslationState::executeTranslationCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::TranslationAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::TranslationAction>::SimpleFeedbackCallback());
    emit goToExecuteTranslationState();


}
void TranslationState::executeTranslationCallback(const actionlib::SimpleClientGoalState& state,
                                            const graspit_msgs::TranslationResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    if (!result->successfulMove)
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
