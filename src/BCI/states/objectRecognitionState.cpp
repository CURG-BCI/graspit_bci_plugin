#include "BCI/states/objectRecognitionState.h"
#include "BCI/graspManager.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "graspit_msgs/GetCameraOrigin.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/SoDB.h>
#include "include/graspitCore.h"
#include "include/ivmgr.h"
#include <Inventor/nodes/SoPerspectiveCamera.h>
using bci_experiment::GraspManager;
using namespace graspit_msgs;

ObjectRecognitionState::ObjectRecognitionState(BCIControlWindow *_bciControlWindow,
                                               ControllerSceneManager *_csm,
                                               ros::NodeHandle *n,
                                               QState* parent):
    State("ObjectRecognitionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    recognizeObjectsActionClient("/crui_manager/run_recognition", true)
{
     objectRecognitionView = new ObjectRecognitionView(bciControlWindow->currentFrame);
     objectRecognitionView->hide();

     get_camera_origin = n->serviceClient<graspit_msgs::GetCameraOrigin>("get_camera_origin");

     alexaPub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);

     connect(
         this,
         SIGNAL(clearGB()),
         GraspManager::getInstance(),
         SLOT(clearObjects()));

     connect(
         this,
         SIGNAL(addToWorld(const QString, const QString, const transf )),
         GraspManager::getInstance(),
         SLOT(addToWorld(const QString , const QString, const transf )));
}


void ObjectRecognitionState::onEntryImpl(QEvent *e)
  {

      std_msgs::String str;
      str.data = "";
      alexaPub.publish(str);

    objectRecognitionView->show();
    bciControlWindow->currentState->setText("Object Recognition State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("object_recognition.png"), pipeline_x, 0, 0.0);

    ROS_INFO("CLEARING ALL GRASPABLE BODIES ON ENTRANCE TO OBJECT RECOGNITION STATE");
    emit clearGB();
    ROS_INFO("NOT MAKING CAMERA ORIGIN REQUEST, DONT WANT TO VIEW FROM ORIGIN");

    sendObjectRecognitionRequest();

}

void ObjectRecognitionState::onExitImpl(QEvent *e)
{
    ROS_INFO(" ObjectRecognitionState::onExit(QEvent *e)");
    SoDB::writelock();
   csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
   SoDB::writeunlock();

    delete csm->pipeline;
    csm->next_target=0;
     objectRecognitionView->hide();

    std::cout << "Finished onExit of Object Recognition State." << std::endl;

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
       graspitCore->getIVmgr()->setCameraTransf(t);
       graspitCore->getIVmgr()->getViewer()->getCamera()->pointAt(SbVec3f(0,0,0), SbVec3f(0,0,1));
    }
    else
    {
      ROS_ERROR("Failed to get Camera Origin.");
    }
}

void ObjectRecognitionState::sendObjectRecognitionRequest()
{
    std::cout <<"ObjectRecognitionState::sendObjectRecognitionRequest()"<< std::endl;
    RunObjectRecognitionGoal goal;
    std::cout <<"ABOUT TO SEND GOAL"<< std::endl;
    recognizeObjectsActionClient.sendGoal(goal,  boost::bind(&ObjectRecognitionState::objectRecognitionCallback, this, _1, _2),
                actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleActiveCallback(),
                actionlib::SimpleActionClient<graspit_msgs::RunObjectRecognitionAction>::SimpleFeedbackCallback());
}

void ObjectRecognitionState::objectRecognitionCallback(const actionlib::SimpleClientGoalState& state,
                       const graspit_msgs::RunObjectRecognitionResultConstPtr& result)
{
    std::cout <<"ObjectRecognitionState::objectRecognitionCallbacks"<< std::endl;
    emit clearGB();
    while(graspitCore->getWorld()->getNumGB())
    {
        usleep(10000);
    }
    ROS_INFO("%lu object(s) found", result->object_info.size());
    std::for_each(result->object_info.begin(),
                  result->object_info.end(),
                  boost::bind(&ObjectRecognitionState::addObject, this, _1));
    ROS_INFO("Sucessfully Finished runObjectRecognition Request");
    BCIService::getInstance()->emitFinishedRecognition();
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
    ROS_INFO("Adding Object %s", objectName.toStdString().c_str());

    emit addToWorld(modelName, objectName, object_pose);
}
