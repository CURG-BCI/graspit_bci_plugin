#include "BCI/states/executionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/onlinePlannerController.h"
#include "include/EGPlanner/searchState.h"

ExecutionState::ExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent)
    : State("ExecutionState", parent), bciControlWindow(_bciControlWindow),
      csm(_csm)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();

    grasp_execution_pubisher = n->advertise<graspit_msgs::Grasp>("/graspit/grasps", 5);
}


void ExecutionState::onEntry(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Execution");

    executeGrasp(OnlinePlannerController::getInstance()->getCurrentGrasp());

    csm->clearTargets();
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_execution.png"), -0.7 , 0.7, 0.0);
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -1.0, 0.0, QString("STOP!")));
    t1->active=true;

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToStoppedExecutionState()));

    csm->addTarget(t1);
}

void ExecutionState::onExit(QEvent *e)
{   delete csm->pipeline;
    executionView->hide();
}


void ExecutionState::executeGrasp(const GraspPlanningState * gps)
{
    graspit_msgs::Grasp grasp;
    grasp.object_name = gps->getObject()->getName().toStdString().c_str();
    grasp.epsilon_quality=gps->getEpsilonQuality();
    grasp.volume_quality=gps->getVolume();
    grasp.grasp_id=gps->getAttribute("graspId");

    double dof[gps->getHand()->getNumDOF()];
    const_cast<GraspPlanningState *>(gps)->getPosture()->getHandDOF(dof);
    for(int i = 0; i < gps->getHand()->getNumDOF(); ++i)
    {
       grasp.final_grasp_dof.push_back(dof[i]);
       grasp.pre_grasp_dof.push_back(dof[i]);
    }

    transf finalHandTransform = gps->readPosition()->getCoreTran();

    float tx = finalHandTransform.translation().x() / 1000;
    float ty = finalHandTransform.translation().y() / 1000;
    float tz = finalHandTransform.translation().z() / 1000;
    float rw = finalHandTransform.rotation().w;
    float rx = finalHandTransform.rotation().x;
    float ry = finalHandTransform.rotation().y;
    float rz = finalHandTransform.rotation().z;

    grasp.final_grasp_pose.position.x=tx ;
    grasp.final_grasp_pose.position.y=ty;
    grasp.final_grasp_pose.position.z=tz;
    grasp.final_grasp_pose.orientation.w=rw;
    grasp.final_grasp_pose.orientation.x=rx;
    grasp.final_grasp_pose.orientation.y=ry;
    grasp.final_grasp_pose.orientation.z=rz;

    double moveDist = -50.0;
    transf pregraspHandTransform = (translate_transf(vec3(0,0,moveDist) * gps->getHand()->getApproachTran()) * gps->readPosition()->getCoreTran());
    tx = pregraspHandTransform.translation().x() / 1000;
    ty = pregraspHandTransform.translation().y() / 1000;
    tz = pregraspHandTransform.translation().z() / 1000;
    rw = pregraspHandTransform.rotation().w;
    rx = pregraspHandTransform.rotation().x;
    ry = pregraspHandTransform.rotation().y;
    rz = pregraspHandTransform.rotation().z;

    grasp.pre_grasp_pose.position.x=tx;
    grasp.pre_grasp_pose.position.y=ty;
    grasp.pre_grasp_pose.position.z=tz;
    grasp.pre_grasp_pose.orientation.w=rw;
    grasp.pre_grasp_pose.orientation.x=rx;
    grasp.pre_grasp_pose.orientation.y=ry;
    grasp.pre_grasp_pose.orientation.z=rz;

    grasp_execution_pubisher.publish(grasp);
}
