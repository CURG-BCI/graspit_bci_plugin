#include "BCI/states/graspPlanningState.h"
#include "BCI/graspManager.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "graspit_msgs/GetCameraOrigin.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/SoDB.h>
#include "include/graspitGUI.h"
#include "include/ivmgr.h"
using bci_experiment::GraspManager;
using namespace graspit_msgs;

PlanGraspState::PlanGraspState(BCIControlWindow *_bciControlWindow,
                                               ControllerSceneManager *_csm,
                                               QState* parent):
    State("GraspPlanningState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm)
{
     graspPlanningView = new GraspPlanningView(bciControlWindow->currentFrame);
     graspPlanningView->hide();

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


void PlanGraspState::onEntryImpl(QEvent *e)
  {
    graspPlanningView->show();
    bciControlWindow->currentState->setText("Grasp Planning State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("grasp_planning.png"), -0.3 , 0, 0.0);


    mObject = GraspManager::getInstance()->getCurrentTarget();
    mObject->setMaterial(5);//rubber

    mHand = graspItGUI->getMainWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    mHandObjectState->reset();

    mPlanner = new SimAnnPlanner(mHand);
    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
    mPlanner->setContactType(CONTACT_PRESET);
    //number of steps to run the planner, the planner starts at 30000
    mPlanner->setMaxSteps(35000);

    mPlanner->resetPlanner();
    mPlanner->startPlanner();

    connect(
        mPlanner,
        SIGNAL(complete()),
        this,
        SLOT(onPlannerFinished()));
}


void PlanGraspState::onPlannerFinished()
{
    GraspManager::getInstance()->clearGrasps();
    for(int i=0; i<mPlanner->getListSize(); i++)
    {
        GraspPlanningState* gps = new GraspPlanningState (mPlanner->getGrasp(i));
        gps->addAttribute("testResult", 0);
        gps->addAttribute("testTime", 0);
        gps->addAttribute("graspId", i);
        GraspManager::getInstance()->addGrasp(gps);
    }
    emit_goToGraspSelectionState();
}


void PlanGraspState::onExitImpl(QEvent *e)
{
   ROS_INFO(" ObjectRecognitionState::onExit(QEvent *e)");
   SoDB::writelock();
   csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
   SoDB::writeunlock();

    delete csm->pipeline;
    csm->next_target=0;
    graspPlanningView->hide();
}


