#include "BCI/states/objectRecognitionState.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "get_camera_origin/GetCameraOrigin.h"

using bci_experiment::OnlinePlannerController;
using namespace moveit_trajectory_planner;

ObjectRecognitionState::ObjectRecognitionState(BCIControlWindow *_bciControlWindow,
                                               ControllerSceneManager *_csm,
                                               ros::NodeHandle *n,
                                               QState* parent):
    State("ObjectRecognitionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    recognizeObjectsActionClient("recognize_objections_action", true)
{
     objectRecognitionView = new ObjectRecognitionView(bciControlWindow->currentFrame);
     objectRecognitionView->hide();

     get_camera_origin = n->serviceClient<get_camera_origin::GetCameraOrigin>("get_camera_origin");

     connect(
         this,
         SIGNAL(clearGB()),
         OnlinePlannerController::getInstance(),
         SLOT(clearObjects()));

     connect(
         this,
         SIGNAL(addToWorld(const QString, const QString, const transf )),
         OnlinePlannerController::getInstance(),
         SLOT(addToWorld(const QString , const QString, const transf )));
}


void ObjectRecognitionState::onEntry(QEvent *e)
  {
    objectRecognitionView->show();
    bciControlWindow->currentState->setText("Object Recognition State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("object_recognition.png"), -0.3 , 0, 0.0);

    sendGetCameraOriginRequest();
    sendObjectRecognitionRequest();

//    if(OnlinePlannerController::getInstance()->hasRecognizedObjects())
//    {
//        BCIService::getInstance()->emitGoToNextState1();
//    }
}

void ObjectRecognitionState::onExit(QEvent *e)
{  delete csm->pipeline;

     objectRecognitionView->hide();
}

void ObjectRecognitionState::sendGetCameraOriginRequest()
{
    get_camera_origin::GetCameraOrigin srv;
    if ( get_camera_origin.call(srv))
    {
       ROS_INFO("Successfully received Camera Origin.");
       float x = srv.response.camera_pose.position.x;
       float y = srv.response.camera_pose.position.y;
       float z = srv.response.camera_pose.position.z;
       transf t(mat3::IDENTITY, 1.5*vec3(x,y,z));
       graspItGUI->getIVmgr()->setCameraTransf(t);
       graspItGUI->getIVmgr()->getViewer()->getCamera()->pointAt(SbVec3f(0,0,0), SbVec3f(0,0,1));
    }
    else
    {
      ROS_ERROR("Failed to get Camera Origin.");
    }
}

void ObjectRecognitionState::sendObjectRecognitionRequest()
{
    RunObjectRecognitionGoal goal;

    recognizeObjectsActionClient.sendGoal(goal,  boost::bind(&ObjectRecognitionState::objectRecognitionCallback, this, _1, _2),
                actionlib::SimpleActionClient<moveit_trajectory_planner::RunObjectRecognitionAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<moveit_trajectory_planner::RunObjectRecognitionAction>::SimpleFeedbackCallback());
}

void ObjectRecognitionState::objectRecognitionCallback(const actionlib::SimpleClientGoalState& state,
                       const moveit_trajectory_planner::RunObjectRecognitionResultConstPtr& result)
{
    emit clearGB();
    while(graspItGUI->getIVmgr()->getWorld()->getNumGB())
    {
        usleep(10000);
    }

    std::for_each(result->object_info.begin(),
                  result->object_info.end(),
                  boost::bind(&ObjectRecognitionState::addObject, this, _1));

    ROS_INFO("Sucessfully Finished runObjectRecognition Request");
     BCIService::getInstance()->emitGoToNextState1();
}

void ObjectRecognitionState::addObject(graspit_msgs::ObjectInfo object)
{
    QString  modelName(QString::fromStdString(object.model_name) + ".xml");
    QString objectName(QString::fromStdString(object.object_name));

    transf object_pose = transf(
                Quaternion(
                    object.object_pose.orientation.w,
                    object.object_pose.orientation.x,
                    object.object_pose.orientation.y,
                    object.object_pose.orientation.z),
                vec3(
                    object.object_pose.position.x*1000.0,
                    object.object_pose.position.y*1000.0,
                    object.object_pose.position.z*1000.0
                    ));

    ROS_INFO("Adding Model %s", modelName.toStdString().c_str());
    ROS_INFO("Adding Model %s", objectName.toStdString().c_str());

    emit addToWorld(modelName, objectName, object_pose);
}
