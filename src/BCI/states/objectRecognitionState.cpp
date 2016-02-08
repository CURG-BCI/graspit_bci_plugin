#include "BCI/states/objectRecognitionState.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "graspit_msgs/GetCameraOrigin.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/SoDB.h>
#include "include/graspitGUI.h"
#include "include/ivmgr.h"
#include <Inventor/nodes/SoPerspectiveCamera.h>
using bci_experiment::OnlinePlannerController;
using namespace graspit_msgs;

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

     get_camera_origin = n->serviceClient<graspit_msgs::GetCameraOrigin>("get_camera_origin");

     n->getParam("use_hardware", use_hardware);
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
    if(use_hardware)
    {
        ROS_INFO("CLEARING ALL GRASPABLE BODIES ON ENTRANCE TO OBJECT RECOGNITION STATE");
        emit clearGB();
        ROS_INFO("NOT MAKEING CAMERA ORIGIN REQUEST, DONT WANT TO VIEW FROM ORIGIN");
        //sendGetCameraOriginRequest();
        sendObjectRecognitionRequest();
    }
    else
    {
          if(OnlinePlannerController::getInstance()->hasRecognizedObjects())
            {
                BCIService::getInstance()->emitGoToNextState1();
            }
    }

}

void ObjectRecognitionState::onExit(QEvent *e)
{                  SoDB::writelock();
                   csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
                   SoDB::writeunlock();
    delete csm->pipeline;
    csm->next_target=0;
     objectRecognitionView->hide();
}

void ObjectRecognitionState::sendGetCameraOriginRequest()
{
    graspit_msgs::GetCameraOrigin srv;
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
                actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleFeedbackCallback());
}

void ObjectRecognitionState::objectRecognitionCallback(const actionlib::SimpleClientGoalState& state,
                       const graspit_msgs::RunObjectRecognitionResultConstPtr& result)
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
    QString  modelName(QString::fromStdString(object.model_name));
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
