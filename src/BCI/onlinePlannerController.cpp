//#include "BCI/onlinePlannerController.h"

//#include "BCI/bciService.h"
//#include "include/debug.h"
//#include "include/EGPlanner/graspTesterThread.h"
//#include "include/EGPlanner/searchState.h"
//#include "src/DBase/graspit_db_model.h"
//#include "src/DBase/graspit_db_grasp.h"
//#include <boost/thread.hpp>
//using bci_experiment::world_element_tools::getWorld;

//class GraspPlanningState;

//namespace bci_experiment
//{


//bool isMainThread(QObject * obj)
//{
//    if(obj->thread() != graspitCore->getIVmgr()->thread())
//    {
//        DBGA("Object not in main thread");
//        return false;
//    }

//    if(QThread::currentThread() != graspitCore->getIVmgr()->thread())
//    {
//        DBGA("Current thread is not main thread");
//        return false;
//    }
//    return true;
//}

//void disableShowContacts()
//{
//    for(int i = 0; i < getWorld()->getNumBodies(); ++i)
//    {
//        getWorld()->getBody(i)->showFrictionCones(false);
//    }
//}

//QMutex OnlinePlannerController::createLock;
//OnlinePlannerController * OnlinePlannerController::onlinePlannerController = NULL;

//OnlinePlannerController* OnlinePlannerController::getInstance()
//{
//    QMutexLocker lock(&createLock);
//    if(!onlinePlannerController)
//    {
//        onlinePlannerController = new OnlinePlannerController();
//        onlinePlannerController->start();
//    }

//    return onlinePlannerController;
//}



//OnlinePlannerController::OnlinePlannerController(QObject *parent) :
//    QThread(parent),
//    mDbMgr(NULL),
//    currentTarget(NULL),
//    currentGraspIndex(0),
//    graspDemonstrationHand(NULL),
//    renderPending(false),
//    sceneLocked(false)
//{
//    mPlanner = planner_tools::createDefaultPlanner();
//    mReachabilityAnalyzer = new ReachabilityAnalyzer();

//    connect(mReachabilityAnalyzer, SIGNAL(updateGraspReachability(int , bool )), this, SLOT(analyzeNextGraspReachabilityCallback(int , bool )));
//}

//bool OnlinePlannerController::analyzeApproachDir()
//{
//    Hand * refHand(mPlanner->getSeedHand());
//    GraspPlanningState * graspPlanningState = new GraspPlanningState(refHand);

//    graspPlanningState->setPostureType(POSE_DOF, false);
//    graspPlanningState->saveCurrentHandState();
//    return true;
//}


//void OnlinePlannerController::showRobots(bool show)
//{
//    if(mPlanner)
//    {
//        mPlanner->getHand()->setRenderGeometry(show);
//        mPlanner->showClone(show);
//        mPlanner->showSolutionClone(show);
//        getSolutionHand()->setRenderGeometry(show);
//    }
//}

//void OnlinePlannerController::showSeedHand(bool show)
//{
//    mPlanner->getSeedHand()->setRenderGeometry(show);
//    if(!show)
//    {
//        mPlanner->getSeedHand()->getWorld()->removeElementFromSceneGraph(mPlanner->getSeedHand());
//    }
//}
//void OnlinePlannerController::showMHand(bool show)
//{
//    mPlanner->getHand()->setRenderGeometry(show);
//}
//void OnlinePlannerController::showSolutionHand(bool show)
//{
//    mPlanner->getSolutionClone()->setRenderGeometry(show);
//}

//void
//OnlinePlannerController::plannerTimedUpdate()
//{
//    /* If there is a planner and the planner has found some solutions
//        * do some tests.
//        */
//    if(mPlanner->getListSize())
//    {
//        // Notify someone to analyze the current approach direction
//        //analyzeApproachDir();
//        //analyzeNextGrasp();
//        // If the planner is itself not updating the order of the solution list
//        if(!mPlanner->isRunning())
//        {
//            mPlanner->showGrasp(0);
//        }
//    }
//    BCIService::getInstance()->onPlannerUpdated();
//    if(timedUpdateRunning)
//    {
//        QTimer::singleShot(1000, this, SLOT(plannerTimedUpdate()));
//    }
//}




//void OnlinePlannerController::initializeDbInterface()
//{
//    DBGA("OnlinePlannerController::initializeDbInterface: checking for mDbMgr");
//    if (!mDbMgr)
//    {
//        DBGA("OnlinePlannerController::initializeDbInterface: about to create graspitDBModelAllocator");
//        GraspitDBModelAllocator *graspitDBModelAllocator = new GraspitDBModelAllocator();
//        DBGA("OnlinePlannerController::initializeDbInterface: about to create graspitDBGraspAllocator");
//        GraspitDBGraspAllocator *graspitDBGraspAllocator = new GraspitDBGraspAllocator(mPlanner->getHand());
//        DBGA("OnlinePlannerController::initializeDbInterface: about to connect");
//        mDbMgr = new db_planner::SqlDatabaseManager("localhost", 5432,
//                                                    "postgres","roboticslab","armdb",graspitDBModelAllocator,graspitDBGraspAllocator);
//        DBGA("OnlinePlannerController::initializeDbInterface: inited mDbMgr\n");
//    }

//    DBGA("OnlinePlannerController::initializeDbInterface: checking for mPlanner.getHand()");
//    if(mPlanner->getHand())
//    {
//        std::cout << "about to import grasps from dbmgr." << std::endl;
//        planner_tools::importGraspsFromDBMgr(mPlanner, mDbMgr);
//        std::cout << "Sucessfully imported grasps from dbmgr." << std::endl;
//    }
//    DBGA("OnlinePlannerController::initializeDbInterface: finished");
//}




//void OnlinePlannerController::initializeTarget()
//{
//    disableShowContacts();

//    //start planner


//    DBGA("OnlinePlannerController::initializeTarget: about to reset planner");
//    mPlanner->resetPlanner();

//    // Download grasps from database and load them in to the planner
//    DBGA("OnlinePlannerController::initializeTarget: about to initializeDbInterface");
//    initializeDbInterface();
//    DBGA("OnlinePlannerController::initializeTarget: about to update solution list");
//    mPlanner->updateSolutionList();
//    // Set the hand to it's highest ranked grasp
//    if(mPlanner->getListSize())
//    {
//        if(!mPlanner->getGrasp(0)->execute(mPlanner->getHand()))
//            mPlanner->getGrasp(0)->execute(mPlanner->getSeedHand());
//        else
//            mPlanner->getSeedHand()->setTran(mPlanner->getHand()->getTran());
//    }
//    //targetsOff = getWorld()->collisionsAreOff(mPlanner->getHand(), mPlanner->getHand()->getGrasp()->getObject());
//    // Realign the hand with respect to the object, moving the hand back to its
//    // pregrasp pose. Use the real hand because it's collisions are set appropriately

//    //world_element_tools::realignHand(currentPlanner->getHand());

//    //Now transfer that position to the reference hand.
//    DBGA("OnlinePlannerController::initializeTarget: finished");

//}



//bool OnlinePlannerController::hasRecognizedObjects()
//{
//    return world_element_tools::getWorld()->getNumGB();
//}


//GraspableBody* OnlinePlannerController::getCurrentTarget()
//{
//    if(!currentTarget && getWorld()->getNumGB())
//    {
//        setCurrentTarget(getWorld()->getGB(0));
//    }
//    return currentTarget;
//}

//GraspableBody* OnlinePlannerController::incrementCurrentTarget()
//{
//    GraspableBody *newTarget = world_element_tools::getNextGraspableBody(currentTarget);
//    setCurrentTarget(newTarget);
//    return currentTarget;
//}

//void OnlinePlannerController::setCurrentTarget(GraspableBody *gb)
//{
//    if(currentTarget && currentTarget != gb)
//        disconnect(currentTarget, SIGNAL(destroyed()),this,SLOT(targetRemoved()));

//    if(gb)
//    {
//        currentTarget = gb;

//        mPlanner->getTargetState()->setObject(currentTarget);
//        mPlanner->setModelState(mPlanner->getTargetState());

//        mPlanner->getSeedHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//        mPlanner->getHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//        OnlinePlannerController::getSolutionHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//        mPlanner->getGraspTester()->getHand()->getGrasp()->setObjectNoUpdate(currentTarget);

//        connect(currentTarget, SIGNAL(destroyed()), this, SLOT(targetRemoved()), Qt::QueuedConnection);
//    }

//}


//void OnlinePlannerController::drawGuides()
//{
//    if(!getHand() || !getCurrentTarget())
//        DBGA("OnlinePlannerController::drawGuides::Error - Tried to draw guides with no hand or no target");

//    ui_tools::updateCircularGuides(mPlanner->getSeedHand(), getCurrentTarget());
//}

//void OnlinePlannerController::destroyGuides()
//{
//    ui_tools::destroyGuideSeparator();
//}

//void OnlinePlannerController::alignHand()
//{
//    Hand * alignedHand = mPlanner->getSeedHand();
//    OnlinePlannerController::getInstance()->getCurrentGrasp()->execute(alignedHand);

//    world_element_tools::realignHand(alignedHand);
//}

//bool OnlinePlannerController::setPlannerToStopped()
//{
//    DBGA("setPlannerToStopped");
//    mPlanner->stopPlanner();
//    return true;
//}

//bool OnlinePlannerController::setPlannerToPaused()
//{
//    DBGA("setPlannerToPaused");
//    mPlanner->pausePlanner();
//    return true;
//}

//bool OnlinePlannerController::setPlannerTargets()
//{
//    // Set the target for the planner state
//    mPlanner->getTargetState()->setObject(currentTarget);
//    mPlanner->setModelState(mPlanner->getTargetState());

//    mPlanner->getSeedHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//    mPlanner->getHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//    OnlinePlannerController::getSolutionHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//    mPlanner->getGraspTester()->getHand()->getGrasp()->setObjectNoUpdate(currentTarget);
//}

//bool OnlinePlannerController::plannerCanBeSetToRunning()
//{
//    if(mPlanner->getState()!=READY)
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR mPlanner->getState() != READY");
//        return false;
//    }
//    return true;
//}

//void OnlinePlannerController::setPlannerToRunning()
//{
//    if(plannerCanBeSetToRunning())
//    {
//        mPlanner->startThread();
//    }
//    drawGuides();
//}

//bool OnlinePlannerController::plannerCanBeSetToReady()
//{
//    if(!currentTarget)
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR !currentTarget");
//        return false;
//    }
//    if(mDbMgr)
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR mDbMgr");
//        return false;
//    }
//    if(mPlanner->getState() == READY)
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR mPlanner->getState() == READY");
//        return false;
//    }
//    if (currentTarget != mPlanner->getHand()->getGrasp()->getObject())
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR currentTarget != mPlanner->getHand()->getGrasp()->getObject()");
//        return false;
//    }
//    if(mPlanner->getTargetState()->getObject() != currentTarget)
//    {
//        DBGA("OnlinePlannerController::setPlannerToReady: ERROR mPlanner->getTargetState()->getObject() != currentTarget");
//        return false;
//    }
//    return true;
//}


//void OnlinePlannerController::setPlannerToReady()
//{
//    DBGA("OnlinePlannerController::setPlannerToReady");

//    //if(plannerCanBeSetToReady())
//    //{
//        DBGA("OnlinePlannerController::setPlannerToReady: About to initialize Target");
//        initializeTarget();
//        DBGA("OnlinePlannerController::setPlannerToReady: About to shutoff collisions between hand and object");
//        getWorld()->collisionsAreOff(mPlanner->getHand(), mPlanner->getHand()->getGrasp()->getObject());
////    }
////    else
////    {
////        DBGA("OnlinePlannerController::setPlannerToReady: ERROR Attempted to set planner to ready");
////    }
//    DBGA("OnlinePlannerController::setPlannerToReady: Success");
//}

//void OnlinePlannerController::rotateHandLong()
//{
//    float stepSize = M_PI/100.0;
//    transf offsetTrans = translate_transf(vec3(0,0,-10));
//    transf robotTran = mPlanner->getSeedHand()->getTran(); //*offsetTrans;
//    transf objectTran = world_element_tools::getCenterOfRotation(currentTarget);


//    transf rotationTrans = (robotTran * objectTran.inverse()) * transf(Quaternion(stepSize, vec3::Z), vec3(0,0,0));
//    transf newTran = rotationTrans *  objectTran;
//    mPlanner->getSeedHand()->moveTo(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);
//    drawGuides();
//}

//void OnlinePlannerController::rotateHandLat()
//{
//    float stepSize = M_PI/100.0;

//    transf robotTran = mPlanner->getSeedHand()->getTran();//*offsetTrans;
//    transf objectTran = world_element_tools::getCenterOfRotation(currentTarget);


//    transf rotationTrans = (robotTran * objectTran.inverse()) * transf(Quaternion(stepSize, vec3::X), vec3(0,0,0));
//    transf newTran = rotationTrans *  objectTran;
//    mPlanner->getSeedHand()->moveTo(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);
//    drawGuides();
//}

//void OnlinePlannerController::incrementGraspIndex()
//{
//    if (mPlanner->getListSize()==0)
//    {
//        return;
//    }
//    currentGraspIndex = (currentGraspIndex + 1)%(mPlanner->getListSize());
//}

//void OnlinePlannerController::decrementGraspIndex()
//{
//    if (mPlanner->getListSize()==0)
//    {
//        return;
//    }
//    currentGraspIndex = currentGraspIndex - 1;
//    if (currentGraspIndex < 0)
//    {
//       currentGraspIndex = mPlanner->getListSize()-1;
//    }
//}

//Hand * OnlinePlannerController::getSeedHand()
//{
//    return mPlanner->getSeedHand();
//}


//Hand * OnlinePlannerController::getHand()
//{
//    return mPlanner->getHand();
//}

//bool isCloneOf(Robot * r1, Robot * r2)
//{
//    return r1->getBase()->getIVGeomRoot()->getChild(0) == r2->getBase()->getIVGeomRoot()->getChild(0);
//}

//Hand * OnlinePlannerController::getSolutionHand()
//{
//    if (!mPlanner)
//    {
//        DBGA("OnlinePlannerController::getGraspDemoHand:Attempted to get demonstration hand with no planner set");
//        return NULL;
//    }
//    return mPlanner->getSolutionClone();
//}

//const GraspPlanningState * OnlinePlannerController::getGrasp(int index)
//{
//    if(mPlanner->getListSize() > 0 && index < mPlanner->getListSize())
//    {
//        return mPlanner->getGrasp(index);
//    }
//    return NULL;
//}

//void OnlinePlannerController::resetGraspIndex()
//{
//    DBGA("OnlinePlannerController::resetGraspIndex");
//    currentGraspIndex = 0;
//}

//unsigned int OnlinePlannerController::getNumGrasps()
//{
//    if (mPlanner)
//        return mPlanner->getListSize();
//    return 0;
//}

//const GraspPlanningState * OnlinePlannerController::getCurrentGrasp()
//{
//    return getGrasp(currentGraspIndex);
//}

//const GraspPlanningState * OnlinePlannerController::getNextGrasp()
//{
//    int index = currentGraspIndex + 1;
//    if (index >= mPlanner->getListSize())
//    {
//        index = 0;
//    }

//    return getGrasp(index);
//}

//bool OnlinePlannerController::stopTimedUpdate()
//{
//    timedUpdateRunning = false;
//    return false;
//}

//bool OnlinePlannerController::startTimedUpdate()
//{
//    timedUpdateRunning = true;
//    plannerTimedUpdate();
//    return true;
//}

//bool OnlinePlannerController::toggleTimedUpdate()
//{
//    if(timedUpdateRunning)
//        stopTimedUpdate();
//    else
//        startTimedUpdate();
//    return timedUpdateRunning;
//}

//bool OnlinePlannerController::startGraspReachabilityAnalysis()
//{
//    DBGA("OnlinePlannerController::startGraspReachabilityAnalysis");
//    mReachabilityAnalyzer->setIsRunning(true);
//    analyzeNextGraspReachability();
//}

//bool OnlinePlannerController::stopGraspReachabilityAnalysis()
//{
//    DBGA("OnlinePlannerController::stopGraspReachabilityAnalysis");
//    mReachabilityAnalyzer->setIsRunning(false);
//}

//void OnlinePlannerController::analyzeNextGraspReachabilityCallback(int graspId, bool isReachable)
//{
//    ROS_INFO("OnlinePlannerController::analyzeNextGraspReachabilityCallback");
//    if(!mReachabilityAnalyzer->isRunning())
//        return;

//    QString attribute = QString("testResult");

//    boost::mutex::scoped_lock lock(mPlanner->mListAttributeMutex);
//    ROS_INFO_STREAM("OnlinePlannerController::analyzeNextGraspReachabilityCallback planner list size" <<  mPlanner->getListSize() << std::endl);
//    for(int i = 0; i < mPlanner->getListSize(); i++ )
//    {
//        const GraspPlanningState * gps = mPlanner->getGrasp(i);
//        if (gps->getAttribute("graspId") == graspId)
//        {

//            int reachabilityScore = 0;
//            if(isReachable)
//            {
//                reachabilityScore = 1;
//            }
//            else
//            {
//                reachabilityScore = -1;
//            }

//            mPlanner->setGraspAttribute(i, attribute, reachabilityScore);
//            ROS_INFO_STREAM("OnlinePlannerController::analyzeNextGraspReachabilityCallback "  << "SetGraspAttribute graspId " << graspId << " attributeString: " << reachabilityScore << "\n");
//            break;
//        }
//    }
//    BCIService::getInstance()->onPlannerUpdated();
//    analyzeNextGraspReachability();
//}

//void OnlinePlannerController::analyzeNextGraspReachability()
//{
//    DBGA("OnlinePlannerController::analyzeNextGraspReachability");

//    if(!mReachabilityAnalyzer->isRunning())
//    {
//        DBGA("OnlinePlannerController::analyzeNextGraspReachability() analysis blocked");
//        return;
//    }

//    if(!mPlanner)
//    {
//        DBGA("OnlinePlannerController::analyzeNextGrasp:: Attempted to analyze grasp with no planner set");
//        return;
//    }

//    int firstUnevaluatedIndex = -1;
//    float currentTime = QDateTime::currentDateTime().toTime_t();
//    float expirationTime =  currentTime - 10;
//    const GraspPlanningState * graspToEvaluate = NULL;
//    // Lock planner's grasp list
//    {
////        boost::mutex::scoped_lock lock(mPlanner->mListAttributeMutex, boost::try_to_lock);
////        if(lock)
////        {
////            DBGA("OnlinePlannerController::analyzeNextGraspReachability() Failed to take lock.");
////            return;
////        }
////        DBGA("OnlinePlannerController::analyzeNextGraspReachability() Took the planner lock");

//        //Check if any test is still pending
//        //Go through all grasps
//        //Ordering of grasps may have changed based on the demonstrated hand pose,
//        //so we must examine all grasps to ensure that none of them are currently being evaluated.

//        for(int i = 0; i < mPlanner->getListSize(); ++i)
//        {
//            //If a grasp hasn't been evaluated
//            const GraspPlanningState * gs = mPlanner->getGrasp(i);
//            if(gs->getAttribute("testResult") == 0.0)
//            {
//                if(firstUnevaluatedIndex < 0)
//                {
//                    firstUnevaluatedIndex = i;
//                }
//                //And an evaluation request was emitted for it less than some time ago
//                if(gs->getAttribute("testTime") > expirationTime)
//                {
//                    DBGA("OnlinePlannerController::analyzeNextGrasp::Last attempt to analyze this grasp was too recent. Grasp ID:" << gs->getAttribute("graspId"));
//                    //Don't emit another request to analyze.
//                    return;
//                }
//            }
//        }
//        if (firstUnevaluatedIndex < 0)
//        {
//            DBGA("OnlinePlannerController::analyzeNextGrasp::No unevaluated grasps to analyze");
//            return;
//        }

//        graspToEvaluate = mPlanner->getGrasp(firstUnevaluatedIndex);
//        assert(graspToEvaluate->getAttribute("testResult") == 0.0);
//        //Request analysis and ask to be called gain when analysis is completed.
//        mPlanner->setGraspAttribute(firstUnevaluatedIndex, "testTime",  QDateTime::currentDateTime().toTime_t());
//        DBGA("Emit grasp analysis");
//        mReachabilityAnalyzer->sendCheckGraspReachabilityRequest(graspToEvaluate);
//    }

//    DBGA("checkGraspReachability: " << currentTime -  QDateTime::currentDateTime().toTime_t());
//}

//void OnlinePlannerController::addToWorld(const QString modelname, const QString object_name, const transf object_pose)
//{
//    QString model_filename = modelname + QString(".xml");
//    ROS_INFO("model filename: %s" , model_filename.toStdString().c_str());
//    if(isSceneLocked())
//    {
//        DBGA("OnlinePlannerController::addToWorld::Tried to add objects to locked world");
//        return;
//    }

//    QString body_file = QString(getenv("GRASPIT")) + "/" +  "models/objects/" + model_filename;
//    Body *b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
//    if(!b)
//    {
//        QString body_file = QString(getenv("GRASPIT")) + "/" +  "models/object_database/" + model_filename;
//        b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
//    }

//    if(b)
//    {
//        b->setTran(object_pose);
//        b->setName(object_name);
//        b->setObjectName(object_name);
//    }

//}

//void OnlinePlannerController::clearObjects()
//{
//    if(isSceneLocked())
//    {
//        DBGA("OnlinePlannerController::clearObjects::Tried to remove objects from locked world");
//        return;
//    }
//    if(mPlanner)
//    {
//        boost::mutex::scoped_lock lock(mPlanner->mListAttributeMutex);
//        mPlanner->pausePlanner();
//        mPlanner->resetPlanner();
//    }
//    while(getWorld()->getNumGB() > 0)
//    {
//        getWorld()->destroyElement(getWorld()->getGB(0), true);
//    }


//}

//void OnlinePlannerController::targetRemoved()
//{
//    currentTarget = NULL;
//    getCurrentTarget();
//}

//void OnlinePlannerController::sortGrasps()
//{
//    DBGA("OnlinePlannerController::sortGrasps");
//    mPlanner->updateSolutionList();
//}

//void OnlinePlannerController::run()
//{
//    exec();
//}

//void OnlinePlannerController::emitRender()
//{
//    if(!renderPending)
//    {
//        emit render();
//        renderPending = true;
//    }
//}

//void OnlinePlannerController::clearGrasps()
//{
//    mPlanner->clearSolutions();
//}

//void OnlinePlannerController::addGrasp(GraspPlanningState *gps)
//{
//    mPlanner->addSolution(gps);
//}

//}
