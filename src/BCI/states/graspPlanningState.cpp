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
    int resolution = 5;
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

//    if (samplingTypeBox->currentText() == "Grid") {
//    boxSampling(a,b,c, resolution);
//    samplingTypeBox->currentText() == "Ellipsoid"
    //ellipsoidSampling(a,b,c, resolution);
    cylinderSampling(a,b,c,resolution);

    //this shows the approach arrows for debugging
    mPlanner->showVisualMarkers( true);

//    mPlanner = new SimAnnPlanner(mHand);
//    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

//    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
//    mPlanner->setContactType(CONTACT_PRESET);
//    //number of steps to run the planner, the planner starts at 30000
//    mPlanner->setMaxSteps(35000);

    mPlanner->resetPlanner();
    mPlanner->startPlanner();

    connect(
        mPlanner,
        SIGNAL(complete()),
        this,
        SLOT(onPlannerFinished()));
}



void PlanGraspState::ellipsoidSampling(double a, double b, double c, double resolution)
{
    //generate a list of grasps by sampling an ellipsoid around the object
    GraspPlanningState seed(mHand);
    seed.setObject(mObject);
    //todo: should use bbox center as reference frame, not object origin
    //which could be anything
    seed.setRefTran(mObject->getTran() * transf(mat3::IDENTITY, vec3(0,0,c)), false);
    seed.setPostureType(POSE_DOF, false);
    seed.setPositionType(SPACE_ELLIPSOID, false);
    seed.reset();

    //set ellipsoid parameters
    seed.getPosition()->setParameter("a", a);
    seed.getPosition()->setParameter("b", b);
    seed.getPosition()->setParameter("c", c);
    //we don't want to sample distance
    seed.getPosition()->getVariable("dist")->setValue(0.0);
    seed.getPosition()->getVariable("dist")->setFixed(true);
    //create a list of earch states that sample the variables of the seed
    std::list<GraspPlanningState*> sampling;
    //uniform sampling of variables. Creates a horrible sampling.
    //createPositionSpaceSampling(seed, &sampling, resolution);
    //grid based sampling. Does somewhat better.
    gridEllipsoidSampling(seed, &sampling, resolution);
    DBGA("Sampled " << sampling.size() << " states.");
    //pass the list to the planner which will take ownership of it and destroy it
    mPlanner->setInput(sampling);
}

void PlanGraspState::addCartesianSamples(const GraspPlanningState &seed,
                                         std::list<GraspPlanningState*> *sampling,
                                         int samples, double x, double y, double z)
{
    //redundant, but easier...
    double a = seed.readPosition()->getParameter("a");
    double b = seed.readPosition()->getParameter("b");
    //double c = seed.readPosition()->getParameter("c");
    //compute angular values
    //from HandObjectStateImpl:
    //x =  a * cos(beta) * cos(gamma);
    //y =  b * cos(beta) * sin(gamma);
    //z =  c * sin(beta);
    double beta = asin(z / sqrt(x*x + y*y + z*z));
    double gamma = atan2(y/b, x/a);
    DBGP("x: " << x << "; y: " << y <<"; z: " << z);
    DBGP("gamma: " << gamma << "; beta: " << beta);
    //sample roll angle as well
    for (int m=0; m<samples; m++) {
        //only sample from 0 to almost PI, as the HH is symmetric
        double tau = M_PI * ((double)m) / samples;
        GraspPlanningState *newState = new GraspPlanningState(&seed);
        newState->getPosition()->getVariable("tau")->setValue(tau);
        newState->getPosition()->getVariable("gamma")->setValue(gamma);
        newState->getPosition()->getVariable("beta")->setValue(beta);
        sampling->push_back(newState);
    }
}

void PlanGraspState::sampleFace(vec3 x, vec3 y, vec3 z, transf seed_ref_tran,
                                double sz1, double sz2, vec3 tln, double res,
                                std::list<GraspPlanningState*> *sampling)
{
    mat3 R(x, y, z);
    int rotSamples=2;

    double m1 = (2.0*sz1 - floor(2.0*sz1 / res) * res)/2.0;
    while (m1 < 2*sz1){
        double m2 = (2.0*sz2 - floor(2.0*sz2 / res) * res)/2.0;
        while (m2 < 2*sz2) {
            vec3 myTln(tln);
            myTln = myTln + (m1 - sz1)* y;
            myTln = myTln + (m2 - sz2)* z;
            transf tr(R, myTln);
            for(int rot=0; rot < rotSamples; rot++) {
                double angle = M_PI * ((double)rot) / rotSamples;
                transf rotTran(Quaternion(angle, vec3(1,0,0)), vec3(0,0,0));
                tr = rotTran * tr;
                GraspPlanningState* seed = new GraspPlanningState(mHand);
                seed->setObject(mObject);
                seed->setRefTran(seed_ref_tran, false);
                seed->setPostureType(POSE_DOF, false);
                seed->setPositionType(SPACE_COMPLETE, false);
                seed->reset();
                seed->getPosition()->setTran(tr);
                sampling->push_back(seed);
            }
            m2+=res;
        }
        m1 += res;
    }
}

void PlanGraspState::boxSampling(double a, double b, double c, double res)
{
    std::list<GraspPlanningState*> sampling;
    transf ref_tran = mObject->getTran()* transf(mat3::IDENTITY, vec3(0,0,c));
    res = 30;
    sampleFace( vec3(0,1,0), vec3(-1,0,0), vec3(0,0,1), ref_tran, a, c, vec3(0,-b,0), res, &sampling);
    sampleFace( vec3(0,-1,0), vec3( 1,0,0), vec3(0,0,1), ref_tran, a, c, vec3(0, b,0), res, &sampling);

    sampleFace( vec3(0,0, 1), vec3(0,1,0), vec3(-1,0,0), ref_tran, b, a, vec3(0,0,-c), res, &sampling);
    sampleFace( vec3(0,0,-1), vec3(0,1,0), vec3( 1,0,0), ref_tran, b, a, vec3(0,0, c), res, &sampling);

    sampleFace( vec3( 1,0,0), vec3(0, 1,0), vec3(0,0,1), ref_tran, b, c, vec3(-a,0,0), res, &sampling);
    sampleFace( vec3(-1,0,0), vec3(0,-1,0), vec3(0,0,1), ref_tran, b, c, vec3( a,0,0), res, &sampling);

    DBGA("Sampled " << sampling.size() << " states.");
    mPlanner->setInput(sampling);
}


void PlanGraspState::gridEllipsoidSampling(const GraspPlanningState &seed,
                                           std::list<GraspPlanningState*> *sampling,
                                           int samples)
{
    double a = seed.readPosition()->getParameter("a");
    double aRes = 2.0 * a / samples;
    double b = seed.readPosition()->getParameter("b");
    double bRes = 2.0 * b / samples;
    double c = seed.readPosition()->getParameter("c");
    double cRes = 2.0 * c / samples;
    DBGP("a: " << a << "; b: " << b <<"; c: " << c);

    for (double i=0.5; i<samples; i+=1.0) {
        for(double j=0.5; j<samples; j+=1.0) {
            addCartesianSamples(seed, sampling, samples,  a, -b+i*bRes, -c+j*cRes);
            addCartesianSamples(seed, sampling, samples, -a, -b+i*bRes, -c+j*cRes);
            addCartesianSamples(seed, sampling, samples, -a+i*aRes, b , -c+j*cRes);
            addCartesianSamples(seed, sampling, samples, -a+i*aRes,-b , -c+j*cRes);
            addCartesianSamples(seed, sampling, samples, -a+i*aRes, -b+j*bRes , c);
            addCartesianSamples(seed, sampling, samples, -a+i*aRes, -b+j*bRes ,-c);
        }
    }
}

void PlanGraspState::cylinderSampling(double a, double b, double c, double res)
{
    std::list<GraspPlanningState*> sampling;

    transf ref_tran = mObject->getTran();

    if (a > b && a > c) {
        sampleCylinderSlices(ref_tran, 0, a, std::max(b,c), res, &sampling);
    } else if (b > a && b > c) {
        sampleCylinderSlices(ref_tran, 1, b, std::max(a,c), res, &sampling);
    } else {
        sampleCylinderSlices(ref_tran, 2, c, std::max(a,b), res, &sampling);
    }

    DBGA("Sampled " << sampling.size() << " states.");
    mPlanner->setInput(sampling);
}
void PlanGraspState::sampleCylinderSlices(transf seed_ref_tran, int axis,
                                double length, double radius, double res,
                                std::list<GraspPlanningState*> *sampling)
{
    double startSample = 2.0 * length / res; // divide cylinder into res, and start sampling from first step as offset (skip boundaries as possibilities)
    double lengthSample = (2.0*length - 2*startSample)/res; // sample step size

    vec3 rotAxis; //TODO: pass in as arg?
    if (axis == 0) {
        std::cout << "------------rotate around x" << std::endl;
        rotAxis = vec3(1,0,0);
    } else if (axis == 1) {
        std::cout << "------------rotate around y" << std::endl;
        rotAxis = vec3(0,1,0);
    } else {
        std::cout << "------------rotate around z" << std::endl;
        rotAxis = vec3(0,0,1);
    }

    // sample along length of cylinder
    for (int i = 0; i <= res  ; i++) {
        // sample around slice circle
        for (int j = 0; j < res; j++) {

            double angle = 2 * M_PI * j/res;
            double x = (radius) * cos(angle);
            double y = (radius) * sin(angle);
            double z = startSample + i * lengthSample;

            transf rot1, rot2;
            if (axis == 0) {
                // rotate around X axis
                rot1 = transf(Quaternion(90*M_PI/180, vec3(0,0,1)), vec3(z, -radius, 0)); // rotate vector to perpendicular from axis and move out to radius
                rot2 = transf(Quaternion(angle, rotAxis), vec3(0,0,0)); // sample around circle
            } else if (axis == 1) {
                // rotate around Y axis
                rot1 = transf(Quaternion(90*M_PI/180, vec3(1,0,0)), vec3(0, z, -radius));
                rot2 = transf(Quaternion(angle, rotAxis), vec3(0,0,0));
            } else {
                // rotate around Z axis
                rot1 = transf(Quaternion(90*M_PI/180, vec3(0,1,0)), vec3(-radius, 0, z));
                rot2 = transf(Quaternion(angle, rotAxis), vec3(0,0,0));
            }
            transf tr = rot1 * rot2;

            GraspPlanningState* seed = new GraspPlanningState(mHand);
            seed->setObject(mObject);
            seed->setRefTran(seed_ref_tran, false);
            seed->setPostureType(POSE_DOF, false);
            seed->setPositionType(SPACE_COMPLETE, false);
            seed->reset();
            seed->getPosition()->setTran(tr);
            sampling->push_back(seed);
        }
    }

    // sample once from top center
    transf tr = transf(Quaternion(180*M_PI/180, vec3(0,1,0)), 2.0 * length * rotAxis);
    GraspPlanningState* seed = new GraspPlanningState(mHand);
    seed->setObject(mObject);
    seed->setRefTran(seed_ref_tran, false);
    seed->setPostureType(POSE_DOF, false);
    seed->setPositionType(SPACE_COMPLETE, false);
    seed->reset();
    seed->getPosition()->setTran(tr);
    sampling->push_back(seed);
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


