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
#include "include/matvecIO.h"
using bci_experiment::GraspManager;
using namespace graspit_msgs;

PlanGraspState::PlanGraspState(BCIControlWindow *_bciControlWindow,
                                               ControllerSceneManager *_csm,
                                               ros::NodeHandle *n,
                                               QState* parent):
    State("GraspPlanningState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    n(n)
{
     graspPlanningView = new GraspPlanningView(bciControlWindow->currentFrame);
     graspPlanningView->hide();

    alexaPub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
}


void PlanGraspState::onEntryImpl(QEvent *e)
{

    std_msgs::String str;
    str.data = "";
    alexaPub.publish(str);
    n->getParam("/experiment_type", experiment_type);

    std::cout << "___>" << experiment_type << std::endl;

    graspPlanningView->show();
    bciControlWindow->currentState->setText("Grasp Planning State");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("grasp_planning.png"), pipeline_x, 0, 0.0);

    mObject = GraspManager::getInstance()->getCurrentTarget();
    mObject->setMaterial(5);//rubber

    std::cout << "object tran:" << std::endl;
    std::cout << mObject->getTran() << std::endl;
    //std::cout << mObject->getIVTran()->rotation << std::endl;
    //std::cout << mObject->getIVTran()->translation << std::endl;

    mHand = graspitCore->getWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    mPlanner = new BCIListPlanner(mHand);

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

    blockSampling(a,b,c);

    //this shows the approach arrows for debugging
    mPlanner->showVisualMarkers(true);

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
    // [ GraspPlanState ] addNewGrasp tr: Position: (0, 0, 80) Orientation: (0, 0, 1, 0)
    std::cout << "[ GraspPlanState ] addNewGrasp tr: Position: (" << tr.translation().x() << ", " << tr.translation().y() << ", " << tr.translation().z() << ") Orientation: (" << tr.rotation().w << ", " << tr.rotation().x << ", " << tr.rotation().y << ", " << tr.rotation().z << ")" << std::endl;

    std::cout << "Adding new grasp" << tr << std::endl;
    GraspPlanningState* seed = new GraspPlanningState(mHand);
    seed->setObject(mObject);
    seed->setRefTran(mObject->getTran(), false);
    seed->setPostureType(POSE_DOF, false);
    seed->setPositionType(SPACE_COMPLETE, false);
    seed->reset();
    seed->getPosition()->setTran(tr);
    sampling->push_back(seed);

    // [ GraspPlanState ] addNewGrasp seed: Position: (0, 0, 192) Orientation: (0, 0, 0, 1)
    transf tr2 = seed->readPosition()->getCoreTran();
    std::cout << "[ GraspPlanState ] addNewGrasp seed: Position: (" << tr2.translation().x() << ", " << tr2.translation().y() << ", " << tr2.translation().z() << ") Orientation: (" << tr2.rotation().w << ", " << tr2.rotation().x << ", " << tr2.rotation().y << ", " << tr2.rotation().z << ")" << std::endl;

    // [ ReachabilityAnalyzer ] addNewGrasp newguy: Orientation: (0, 0, 1, 0)
    const mat3& affine = tr2.affine();
    std::cout << "[ReachabilityAnalyzer] affineMatrix: \n" << affine << std::endl;
    Quaternion newguy(affine);
    std::cout << "[ ReachabilityAnalyzer ] addNewGrasp newguy: Orientation: (" << newguy.w << ", " << newguy.x << ", " << newguy.y << ", " << newguy.z << ")" << std::endl;
}

void PlanGraspState::blockSampling(double a, double b, double c)
{
    std::list<GraspPlanningState*> sampling;

    vec3 rotAxis = vec3(0,0,1); // rotate around Z axis

    // sample once from top center
    transf tr = transf(Quaternion(0, 0, 1, 0), vec3(0,0,80));
    addNewGrasp(tr, &sampling);

    std::cout << "[ GraspPlanState ] blockSampling tr: Position: (" << tr.translation().x() << ", " << tr.translation().y() << ", " << tr.translation().z() << ") Orientation: (" << tr.rotation().w << ", " << tr.rotation().x << ", " << tr.rotation().y << ", " << tr.rotation().z << ")" << std::endl;

//    transf tr2 = transf(Quaternion(M_PI/2.0, vec3(0,0,1)), vec3(0,0,0));
//    tr2 = tr2 * tr;
//    addNewGrasp(tr2, &sampling);

    std::cout << "size of sampling is " << sampling.size() << std::endl;

    mPlanner->setInput(sampling);
}

void PlanGraspState::onPlannerFinished()
{
    GraspManager::getInstance()->clearGrasps();
    std::cout << "mPlanner list size " << mPlanner->getListSize() << std::endl;
    for(int i=0; i<mPlanner->getListSize(); i++)
    {
        GraspPlanningState* gps = new GraspPlanningState (mPlanner->getGrasp(i));
        gps->addAttribute("testResult", 0);
        gps->addAttribute("testTime", 0);
        gps->addAttribute("graspId", i);
        GraspManager::getInstance()->addGrasp(gps);

        transf tr = gps->readPosition()->getCoreTran();

        std::cout << "[ GraspPlanningState ] OnPlannerFinished: Position: (" << tr.translation().x() << ", " << tr.translation().y() << ", " << tr.translation().z() <<
                                                            ") Orientation: (" << tr.rotation().w << ", " << tr.rotation().x << ", " << tr.rotation().y << ", " << tr.rotation().z << ")" << std::endl;

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

    //mPlanner->showVisualMarkers(false);

    graspPlanningView->hide();

    mObject = NULL;

    //delete mPlanner;
}
