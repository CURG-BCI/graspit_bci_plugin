#include "BCI/states/executeGoToBinState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/AfterGraspingGoal.h"

ExecuteGoToBinState::ExecuteGoToBinState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent)
    : State("ExecuteGoToBinState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      afterGraspingActionClient("after_grasping_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();

    grasp_stop_execution_publisher = n->advertise<actionlib_msgs::GoalID>("/mico_arm_driver/controller/follow_joint_trajectory/cancel", 5);
    grasp_stop_fingers_execution_publisher = n->advertise<actionlib_msgs::GoalID>("/mico_arm_driver/fingers/finger_positions/cancel", 5);
}


void ExecuteGoToBinState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Execute Go Home");
    goToBin();

    csm->clearTargets();
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       -1.4, -1.0, 0.0, QString("STOP!")));
    t1->active=true;

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToStoppedGoToBinState()));

    csm->addTarget(t1);
}

void ExecuteGoToBinState::emit_goToStoppedGoToBinState()
{
    afterGraspingActionClient.cancelAllGoals();
    actionlib_msgs::GoalID goal;

    grasp_stop_execution_publisher.publish(goal);
    usleep(10000);
    grasp_stop_execution_publisher.publish(goal);
    usleep(10000);
    grasp_stop_execution_publisher.publish(goal);
    usleep(10000);

    grasp_stop_fingers_execution_publisher.publish(goal);
    usleep(10000);
    grasp_stop_fingers_execution_publisher.publish(goal);
    usleep(10000);
    grasp_stop_fingers_execution_publisher.publish(goal);
    usleep(10000);

    emit goToStoppedGoToBinState();
}

void ExecuteGoToBinState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Execute Move Object State." << std::endl;
}


void ExecuteGoToBinState::goToBin()
{
    graspit_msgs::AfterGraspingGoal goal;
    goal.action = "moveobject";
    afterGraspingActionClient.sendGoal(goal, boost::bind(&ExecuteGoToBinState::goToBinCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::AfterGraspingAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::AfterGraspingAction>::SimpleFeedbackCallback());


}
void ExecuteGoToBinState::goToBinCallback(const actionlib::SimpleClientGoalState& state,
                                            const graspit_msgs::AfterGraspingResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    emit goToHomeState();
}
