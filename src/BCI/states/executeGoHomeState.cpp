#include "BCI/states/executeGoHomeState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/AfterGraspingGoal.h"

ExecuteGoHomeState::ExecuteGoHomeState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent)
    : State("ExecuteGoHomeState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      afterGraspingActionClient("after_grasping_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();

    grasp_stop_execution_publisher = n->advertise<actionlib_msgs::GoalID>("/mico_arm_driver/controller/follow_joint_trajectory/cancel", 5);
    grasp_stop_fingers_execution_publisher = n->advertise<actionlib_msgs::GoalID>("/mico_arm_driver/fingers/finger_positions/cancel", 5);
}


void ExecuteGoHomeState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Execute Go Home");
    goHome();

    csm->clearTargets();
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       btn_x, btn_y, 0.0, QString("STOP!")));
    t1->active=true;

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToStoppedGoHomeState()));

    csm->addTarget(t1);
}

void ExecuteGoHomeState::emit_goToStoppedGoHomeState()
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

    emit goToStoppedGoHomeState();
}

void ExecuteGoHomeState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Execute Go Home State." << std::endl;
}


void ExecuteGoHomeState::goHome()
{
    graspit_msgs::AfterGraspingGoal goal;
    goal.action = "gohome";
    afterGraspingActionClient.sendGoal(goal, boost::bind(&ExecuteGoHomeState::goHomeCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::AfterGraspingAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::AfterGraspingAction>::SimpleFeedbackCallback());


}
void ExecuteGoHomeState::goHomeCallback(const actionlib::SimpleClientGoalState& state,
                                        const graspit_msgs::AfterGraspingResultConstPtr& result)
{
    std::cout << "Action completed" << std::endl;
    emit goToHomeState();
}
