#include "BCI/states/graspPlanningState.h"
#include "BCI/graspManager.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "graspit_msgs/GetCameraOrigin.h"
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/SoDB.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include "include/graspitCore.h"
#include "include/ivmgr.h"
#include "listPlanner.h"
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
}


void PlanGraspState::onEntryImpl(QEvent *e)
  {
    graspPlanningView->show();
    bciControlWindow->currentState->setText("Grasp Planning State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("grasp_planning.png"), -0.3 , 0, 0.0);


    mObject = GraspManager::getInstance()->getCurrentTarget();
    mObject->setMaterial(5);//rubber

    mHand = graspitCore->getWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    mHandObjectState->reset();

    mPlanner = new ListPlanner(mHand);
    //get object bbox dimensions
    SoGetBoundingBoxAction *bba =
        new SoGetBoundingBoxAction(graspitCore->getIVmgr()->getViewer()->getViewportRegion());
    bba->apply(mObject->getIVGeomRoot());
    SbVec3f bbmin,bbmax;
    bba->getBoundingBox().getBounds(bbmin,bbmax);
    delete bba;
    double a = 0.5*(bbmax[0] - bbmin[0]);
    double b = 0.5*(bbmax[1] - bbmin[1]);
    double c = 0.5*(bbmax[2] - bbmin[2]);

    cylinderSampling(a,b,c,4,12);

    //this shows the approach arrows for debugging
    mPlanner->showVisualMarkers( true);

    mPlanner->resetPlanner();
    mPlanner->startPlanner();

    connect(
        mPlanner,
        SIGNAL(complete()),
        this,
        SLOT(onPlannerFinished()));
}

void PlanGraspState::addNewGrasp(transf tr, std::list<GraspPlanningState*> *sampling)
{
    GraspPlanningState* seed = new GraspPlanningState(mHand);
    seed->setObject(mObject);
    seed->setRefTran(mObject->getTran(), false);
    seed->setPostureType(POSE_DOF, false);
    seed->setPositionType(SPACE_COMPLETE, false);
    seed->reset();
    seed->getPosition()->setTran(tr);
    sampling->push_back(seed);
}

void PlanGraspState::cylinderSampling(double a, double b, double c, double resLength, double resRotation)
{
    std::list<GraspPlanningState*> sampling;

    // object model is always right side up along z axis
    double length = c;
    double radius = std::max(a,b);

    resLength += 1; // divide length by resLength + 1 so there are resLength samples and not including edge of bbox
    double lengthSample = 2.0 * length / resLength;

    vec3 rotAxis = vec3(0,0,1); // rotate around Z axis

    // sample along length of cylinder
    for (int i = 1; i < resLength; i++) {
        // sample around slice circle
        for (int j = 0; j < resRotation; j++) {

            double angle = 2 * M_PI * j/resRotation;
            double x = (radius) * cos(angle);
            double y = (radius) * sin(angle);
            double z = i * lengthSample;

            transf rot1 = transf(Quaternion(90*M_PI/180, vec3(0,1,0)), vec3(-radius, 0, z)); // rotate vector to be perpendicular to z axis
            transf rot2 = transf(Quaternion(angle, rotAxis), vec3(0,0,0)); // rotate around axis to sample
            transf tr = rot1 * rot2;

            addNewGrasp(tr, &sampling);
        }
    }

    // sample once from top center
    transf tr = transf(Quaternion(180*M_PI/180, vec3(0,1,0)), 2.1 * length * rotAxis);
    addNewGrasp(tr, &sampling);

    // sample once from bottom center
    tr = transf(Quaternion(0, vec3(0,1,0)), 0 * rotAxis);
    addNewGrasp(tr, &sampling);

    DBGA("Sampled " << sampling.size() << " states.");
    mPlanner->setInput(sampling);
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
    mObject = NULL;
}


