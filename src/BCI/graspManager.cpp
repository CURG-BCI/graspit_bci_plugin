#include "BCI/graspManager.h"

#include "BCI/bciService.h"
#include "include/debug.h"
#include "include/EGPlanner/graspTesterThread.h"
#include "include/EGPlanner/searchState.h"
#include "include/DBase/graspit_db_model.h"
#include "include/DBase/graspit_db_grasp.h"
#include <boost/thread.hpp>
using bci_experiment::world_element_tools::getWorld;

class GraspPlanningState;

namespace bci_experiment
{


bool isMainThread(QObject * obj)
{
    if(obj->thread() != graspitCore->getIVmgr()->thread())
    {
        DBGA("Object not in main thread");
        return false;
    }

    if(QThread::currentThread() != graspitCore->getIVmgr()->thread())
    {
        DBGA("Current thread is not main thread");
        return false;
    }
    return true;
}

void disableShowContacts()
{
    for(int i = 0; i < getWorld()->getNumBodies(); ++i)
    {
        getWorld()->getBody(i)->showFrictionCones(false);
    }
}

QMutex GraspManager::createLock;
GraspManager * GraspManager::graspManager = NULL;

GraspManager* GraspManager::getInstance()
{
    QMutexLocker lock(&createLock);
    if(!graspManager)
    {
        graspManager = new GraspManager();
        graspManager->start();
    }

    return graspManager;
}



GraspManager::GraspManager(QObject *parent) :
    QThread(parent),
    mDbMgr(NULL),
    mHand(NULL),
    currentGraspIndex(0),
    currentTargetIndex(0),
    renderPending(false)
{
    mReachabilityAnalyzer = new ReachabilityAnalyzer();

    connect(mReachabilityAnalyzer, SIGNAL(updateGraspReachability(int , bool )), this, SLOT(analyzeNextGraspReachabilityCallback(int , bool )));
}


void GraspManager::showRobots(bool show)
{
    if(!mHand)
    {
        getHand();
    }
    mHand->setRenderGeometry(show);

    if(!show)
        {
            getWorld()->removeElementFromSceneGraph(mHand);
        }
    else
    {
        getWorld()->addElementToSceneGraph(mHand);
    }

}

void GraspManager::showMHand(bool show)
{
    mHand->setRenderGeometry(show);
}



void GraspManager::initializeDbInterface()
{
    DBGA("OnlinePlannerController::initializeDbInterface: checking for mDbMgr");
    if (!mDbMgr)
    {
        DBGA("OnlinePlannerController::initializeDbInterface: about to create graspitDBModelAllocator");
        GraspitDBModelAllocator *graspitDBModelAllocator = new GraspitDBModelAllocator();
        DBGA("OnlinePlannerController::initializeDbInterface: about to create graspitDBGraspAllocator");
        GraspitDBGraspAllocator *graspitDBGraspAllocator = new GraspitDBGraspAllocator(getHand());
        DBGA("OnlinePlannerController::initializeDbInterface: about to connect");
        mDbMgr = new db_planner::SqlDatabaseManager("tonga.cs.columbia.edu", 5432,
                                                    "postgres","roboticslab","armdb",graspitDBModelAllocator,graspitDBGraspAllocator);
        DBGA("OnlinePlannerController::initializeDbInterface: inited mDbMgr\n");
    }

    DBGA("OnlinePlannerController::initializeDbInterface: checking for mPlanner.getHand()");
    if(getHand())
    {
        std::cout << "about to import grasps from dbmgr." << std::endl;
        planner_tools::importGraspsFromDBMgr(getHand(), getCurrentTarget(), mGraspList, mDbMgr);
        std::cout << "Sucessfully imported grasps from dbmgr." << std::endl;
    }
    DBGA("OnlinePlannerController::initializeDbInterface: finished");
}

void GraspManager::run()
{
    exec();
}

void GraspManager::updateSolutionList()
{

    std::vector<GraspPlanningState *> for_deletion;
    std::vector<GraspPlanningState*>::iterator it;
    //re-compute distance between current hand position and solutions.
    for ( it = mGraspList.begin(); it != mGraspList.end(); it++ )
    {

        double dist = (*it)->getEnergy();

        int reachable = 1;
        int unreachable = -1;
        int untested = 0;

        //Ensures that gps has an IVRoot. DO NOT DELETE
        (*it)->getIVRoot();

        if((*it)->getAttribute("testResult") == reachable)
        {
            dist -= 1000;
            (*it)->setIVMarkerColor(1-dist, dist, 0);
        }
        else if ((*it)->getAttribute("testResult") == unreachable)
        {
            (*it)->setIVMarkerColor(0 , 1, 1);
            for_deletion.push_back(*it);
        }

        (*it)->setDistance(dist);

    }

    for(auto delete_it=for_deletion.begin();delete_it!=for_deletion.end();delete_it++)
    {
        for(auto i=mGraspList.begin();i!=mGraspList.end();i++)
        {
            if((*delete_it)==(*i))
            {
                mGraspList.erase(i);
                break;
            }

        }
    }

    //keep only best in list
    std::vector<GraspPlanningState *>::iterator it2 = mGraspList.begin();
    int SOLUTION_BUFFER_SIZE = 10;
    for(int i = 0; it2 != mGraspList.end();)
    {
        if ((*it2)->getAttribute("testResult") <= 0)
        {
            break;
        }

        if(i >= SOLUTION_BUFFER_SIZE)
        {
            delete *it2;
            std::vector<GraspPlanningState *>::iterator it3 = it2;
            ++it2;
            mGraspList.erase(it3);
        }
        else
        {
            ++it2;
        }

        ++i;
    }

    while (mGraspList.size() > 2*SOLUTION_BUFFER_SIZE)
    {
        delete mGraspList.back();
        mGraspList.pop_back();
    }

}


bool GraspManager::hasRecognizedObjects()
{
    return world_element_tools::getWorld()->getNumGB();
}


GraspableBody* GraspManager::getCurrentTarget()
{
    if (getWorld()->getNumGB()==0)
    {
        return NULL;
    }
    mHand->getGrasp()->setObject(getWorld()->getGB(currentTargetIndex));
    return getWorld()->getGB(currentTargetIndex);
}

GraspableBody* GraspManager::incrementCurrentTarget()
{
    if (getWorld()->getNumGB()==0)
    {
        return NULL;
    }
    currentTargetIndex = (currentTargetIndex + 1)%(getWorld()->getNumGB());
    return getCurrentTarget();

}


void GraspManager::getGraspsFromDB()
{
    disableShowContacts();


    initializeDbInterface();

    updateSolutionList();
    // Set the hand to it's highest ranked grasp
    if(mGraspList.size())
    {
        mGraspList.at(0)->execute(getHand());
    }
}

void GraspManager::incrementGraspIndex()
{
    if (mGraspList.size()==0)
    {
        return;
    }
    currentGraspIndex = (currentGraspIndex + 1)%(mGraspList.size());
}

void GraspManager::decrementGraspIndex()
{
    if (mGraspList.size()==0)
    {
        return;
    }
    currentGraspIndex = currentGraspIndex - 1;
    if (currentGraspIndex < 0)
    {
       currentGraspIndex = mGraspList.size()-1;
    }
}


Hand * GraspManager::getHand()
{
    if(!mHand)
    {
        mHand = graspitCore->getWorld()->getCurrentHand();
        mHand->getGrasp()->setObject(graspitCore->getWorld()->getGB(0));
    }
    return mHand;
}


GraspPlanningState * GraspManager::getGrasp(int index)
{
    if(mGraspList.size() > 0 && index < mGraspList.size())
    {
        return mGraspList.at(index);
    }
    return NULL;
}

void GraspManager::resetGraspIndex()
{
    currentGraspIndex = 0;
}

const GraspPlanningState * GraspManager::getCurrentGrasp()
{
    return getGrasp(currentGraspIndex);
}

const GraspPlanningState * GraspManager::getNextGrasp()
{
    int index = currentGraspIndex + 1;
    if (index >= mGraspList.size())
    {
        index = 0;
    }

    return getGrasp(index);
}


bool GraspManager::startGraspReachabilityAnalysis()
{
    DBGA("OnlinePlannerController::startGraspReachabilityAnalysis");
    mReachabilityAnalyzer->setIsRunning(true);
    analyzeNextGraspReachability();
}

bool GraspManager::stopGraspReachabilityAnalysis()
{
    DBGA("OnlinePlannerController::stopGraspReachabilityAnalysis");
    mReachabilityAnalyzer->setIsRunning(false);
}


void GraspManager::analyzeNextGraspReachabilityCallback(int graspId, bool isReachable)
{
    ROS_INFO("OnlinePlannerController::analyzeNextGraspReachabilityCallback");
    if(!mReachabilityAnalyzer->isRunning())
        return;

    QString attribute = QString("testResult");

    for(int i = 0; i < mGraspList.size(); i++ )
    {
        GraspPlanningState * gps = mGraspList.at(i);
        if (gps->getAttribute("graspId") == graspId)
        {

            int reachabilityScore = 0;
            if(isReachable)
            {
                reachabilityScore = 1;
            }
            else
            {
                reachabilityScore = -1;
            }
            gps->setAttribute(attribute, reachabilityScore);
            break;
        }
    }
    BCIService::getInstance()->onPlannerUpdated();
    analyzeNextGraspReachability();
    updateSolutionList();
}

void GraspManager::analyzeNextGraspReachability()
{
    DBGA("OnlinePlannerController::analyzeNextGraspReachability");

    if(!mReachabilityAnalyzer->isRunning())
    {
        DBGA("OnlinePlannerController::analyzeNextGraspReachability() analysis blocked");
        return;
    }


    int firstUnevaluatedIndex = -1;
    float currentTime = QDateTime::currentDateTime().toTime_t();
    float expirationTime =  currentTime - 10;
    GraspPlanningState * graspToEvaluate = NULL;


        for(int i = 0; i < mGraspList.size(); ++i)
        {
            //If a grasp hasn't been evaluated
            GraspPlanningState * gs = getGrasp(i);
            if(gs->getAttribute("testResult") == 0.0)
            {
                if(firstUnevaluatedIndex < 0)
                {
                    firstUnevaluatedIndex = i;
                }
                //And an evaluation request was emitted for it less than some time ago
                if(gs->getAttribute("testTime") > expirationTime)
                {
                    DBGA("OnlinePlannerController::analyzeNextGrasp::Last attempt to analyze this grasp was too recent. Grasp ID:" << gs->getAttribute("graspId"));
                    //Don't emit another request to analyze.
                    return;
                }
            }
        }
        if (firstUnevaluatedIndex < 0)
        {
            DBGA("OnlinePlannerController::analyzeNextGrasp::No unevaluated grasps to analyze");
            return;
        }

        graspToEvaluate = getGrasp(firstUnevaluatedIndex);
        graspToEvaluate->setAttribute("testTime",  QDateTime::currentDateTime().toTime_t());
        assert(graspToEvaluate->getAttribute("testResult") == 0.0);
        //Request analysis and ask to be called gain when analysis is completed.
        DBGA("Emit grasp analysis");
        mReachabilityAnalyzer->sendCheckGraspReachabilityRequest(graspToEvaluate);


    DBGA("checkGraspReachability: " << currentTime -  QDateTime::currentDateTime().toTime_t());
}

void GraspManager::addToWorld(const QString modelname, const QString object_name, const transf object_pose)
{
    QString model_filename = modelname + QString(".xml");
    ROS_INFO("model filename: %s" , model_filename.toStdString().c_str());

    QString body_file = QString(getenv("GRASPIT")) + "/" +  "models/objects/" + model_filename;
    Body *b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
    if(!b)
    {
        QString body_file = QString(getenv("GRASPIT")) + "/" +  "models/object_database/" + model_filename;
        b = graspitCore->getWorld()->importBody("GraspableBody", body_file);
    }

    if(b)
    {
        b->setTran(object_pose);
        b->setName(object_name);
        b->setObjectName(object_name);
    }

}

void GraspManager::clearObjects()
{
    while(getWorld()->getNumGB() > 0)
    {
        getWorld()->destroyElement(getWorld()->getGB(0), true);
    }


}

void GraspManager::emitRender()
{
    if(!renderPending)
    {
        emit render();
        renderPending = true;
    }
}

void GraspManager::clearGrasps()
{
    mGraspList.clear();
}

void GraspManager::addGrasp(GraspPlanningState *gps)
{
    mGraspList.push_back(gps);
}

}
