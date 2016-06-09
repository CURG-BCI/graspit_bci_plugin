#include "BCIOnlinePlanner.h"

#include <Inventor/nodes/SoSeparator.h>
#include <QMutexLocker>
#include "include/world.h"
#include "include/robot.h"
#include "include/EGPlanner/simAnn.h"
#include "include/EGPlanner/searchState.h"
#include "include/EGPlanner/searchEnergy.h"
#include "include/EGPlanner/graspTesterThread.h"
#include "include/EGPlanner/onLineGraspInterface.h"
#include "include/grasp.h"
#include "include/eigenGrasp.h"
#include "include/matvec3D.h"
#include "include/graspitGUI.h"
#include "include/ivmgr.h"
#include "bciService.h"

//#define GRASPITDBG
#include "include/debug.h"

#define SHOW_RECENT_SOLUTION 1
#define CANDIDATE_BUFFER_SIZE 20
#define SOLUTION_BUFFER_SIZE 10

BCIOnlinePlanner::BCIOnlinePlanner(Hand *h) : SimAnnPlanner(h)
{
    mSolutionClone = NULL;
    mMarkSolutions = false;
    mCurrentBest = NULL;
    mSimAnn->setParameters(ANNEAL_ONLINE);
    setRenderType(RENDER_ALWAYS);
    mRepeat = true;

    mGraspTester = new GraspTester(h);
    //mGraspTester->getTargetState()->setObject(h->getGrasp()->getObject());
    mGraspTester->startThread();
    mGraspTester->showClone(false);

    //the on-line planner ALWAYS uses a clone for the search but the original hand is saved as the reference hand
    mSeedHand = h;
    createSolutionClone();
    //createAndUseClone();//after this point mHand now points to a new clone
    //in case that later we might want to see what the clone is doing
    mHand->setRenderGeometry(true);
    //but for now it is hidden
    //showClone(false);
    //hack - I need a better way to handle collisions when the planner is using a clone
    //we have three hands we need to take care of: the original hand, this clone and the parallel tester's clone
    //some of the collisions are turned off by createAndUseClone(), but not this one
}

BCIOnlinePlanner::~BCIOnlinePlanner()
{
    mSeedHand->setTransparency(0);
    mGraspTester->stopPlanner();
    mGraspTester->wait();
    delete mGraspTester;
    if (mSolutionClone)
    {
        mHand->getWorld()->destroyElement(mSolutionClone);
    }
}

void
BCIOnlinePlanner::resetParameters()
{
    SimAnnPlanner::resetParameters();
    if (mCurrentBest) mCurrentBest->setEnergy(1.0e8);
}

void
BCIOnlinePlanner::createSolutionClone()
{
    mSolutionClone = new Hand(mHand->getWorld(), "Solution clone");
    mSolutionClone->cloneFrom(mSeedHand);//CHANGED! was mHand - for some reason this makes setting transparency not tied to mHand??
    mSolutionClone->setTransparency(0.03);//Make the clone that shows the solutions slightly transparent so we can still see the object below it.
    mSolutionClone->showVirtualContacts(false);
    mSolutionClone->setRenderGeometry(true);
    //solution clone is always added to scene graph
    mHand->getWorld()->addRobot(mSolutionClone, true);
    mHand->getWorld()->toggleCollisions(false, mSolutionClone);
    mSolutionClone->setTran( mSeedHand->getTran() );//CHANGED!  was mHand
}


void
BCIOnlinePlanner::showSolutionClone(bool s)
{
    if (s)
    {
        mHand->getWorld()->addElementToSceneGraph(mSolutionClone);
    }
    else
    {
        mHand->getWorld()->removeElementFromSceneGraph(mSolutionClone);
    }
}

void
BCIOnlinePlanner::showClone(bool s)
{
    SimAnnPlanner::showClone(s);
}

bool
BCIOnlinePlanner::resetPlanner()
{
    DBGA("Online planner reset");
    if (!mGraspTester->resetPlanner())
    {
        DBGA("Failed to reset parallel tester!");
        return false;
    }
    while (!mCandidateList.empty())
    {
        delete mCandidateList.front(); mCandidateList.pop_front();
    }
    if (!SimAnnPlanner::resetPlanner())
    {
        return false;
    }
    if (mCurrentBest)
    {
        delete mCurrentBest;
    }
    mCurrentBest = new GraspPlanningState(mCurrentState);
    return true;
}

void BCIOnlinePlanner::createAndUseClone()
{
    SimAnnPlanner::createAndUseClone();
    mHand->getWorld()->toggleCollisions(false, mGraspTester->getHand(), mHand);
    //so we can distinguish between the two clones
    mGraspTester->getHand()->setName( mGraspTester->getHand()->getName() + QString(" th") );//this hand is never put in scene graph, for behind the scenes stuff?
    mHand->setName( mHand->getName() + QString(" pl") );
    mHand->setTransparency(0.8);//Make the planner hand barely visible just so the user can see something is going on.
}

void BCIOnlinePlanner::startThread()
{
    setState(INIT);
    if(!this->mMultiThread)
    {
        SimAnnPlanner::startThread();
    }
    mGraspTester->startPlanner();

    //mSeedHand->setTransparency(0.7);
    //this->mHand->moveToThread(this);
    //mHand->getWorld()->toggleCollisions(true, mHand);
    showClone(false);
    showSolutionClone(true);
    SimAnnPlanner::startPlanner();
    setRenderType(RENDER_LEGAL);
    setState(RUNNING);
    DBGA("Started on-line planner");
}


void
BCIOnlinePlanner::startPlanner()
{
    createAndUseClone();
    mSeedHand->setTransparency(0.7);
    SimAnnPlanner::startPlanner();
    mGraspTester->startPlanner();
    mGraspTester->setRenderType(RENDER_NEVER);

    showClone(true);
    showSolutionClone(true);
    mGraspTester->showClone(true);

    DBGA("Started on-line planner");
}



void
BCIOnlinePlanner::pausePlanner()
{
    DBGA("pausePlanner");
    mGraspTester->pausePlanner();
    msleep(1000);
    SimAnnPlanner::pausePlanner();
    showClone(false);
    mGraspTester->showClone(false);
}

double
BCIOnlinePlanner::stateDistance(const GraspPlanningState *s1, const GraspPlanningState *s2)
{
    return distanceOutsideApproach(s1->getTotalTran(), s2->getTotalTran());
}

/*!	A helper function that gives the change between two transforms, but 
    disregards any change along the approach direction of the hand. I'm not
    really sure this is needed anymore, might be replaced in the future.
*/
double
BCIOnlinePlanner::distanceOutsideApproach(const transf &solTran, const transf &handTran, bool useAlignment)
{
    double max_angle = M_PI / 4.0;
    double max_dist = 50.0;
    double f;
    //relative transform between the two
    transf changeTran = solTran * handTran.inverse();
    transf handApproach = mHand->getApproachTran() * handTran;
    transf solApproachTran = mHand->getApproachTran() * solTran;

    vec3 handApproachZ(handApproach.affine()[2], handApproach.affine()[5], handApproach.affine()[8]);
    vec3 solApproachZ(solApproachTran.affine()[2],solApproachTran.affine()[5], solApproachTran.affine()[8]);

    //DBGP("T1: " << solTran.translation());
    //DBGP("T2: " << handTran.translation());
    //DBGP("Change: " << changeTran.translation() );

    //get change in terms of approach direction
    changeTran = mHand->getApproachTran() * changeTran * mHand->getApproachTran().inverse();
    //double dotZ = (mHand->getApproachTran()*handTran).affine().row(2) * (mHand->getApproachTran()*solTran).affine().transpose().row(2);
    //get angular change
    double angle; vec3 axis;
    changeTran.rotation().ToAngleAxis(angle, axis);

    //get translation change
    vec3 approach = changeTran.translation();
    //change along approach direction does not count as distance
    //DBGP("Approach: " << approach);
    if (approach.z() < 0) {
        f = -1.0;
    } else {
        f = 1.0;
    }
    double angleMod = 1.0;

    if (handApproachZ%solApproachZ < 0)
        angleMod = 10.0;

    approach.z() = 0;
    double dist = approach.len();

    //compute final value
    if (angle > M_PI) angle -= 2*M_PI;
    if (angle < -M_PI) angle += 2*M_PI;
    angle = fabs(angle) / max_angle ;
    dist = dist / max_dist;
    double alignment_score = 1/angleMod;
    //if (useAlignment) alignment_score = .001+(axis.z() * angleMod);

    return f * std::max(angle, dist)/alignment_score;
}


void BCIOnlinePlanner::setGraspAttribute(int i, const QString &attribute, double value)
{
    assert (i>=0 && i<(int)mBestList.size());
    std::list<GraspPlanningState*>::iterator iterator = mBestList.begin();
    std::advance(iterator, i);
    (*iterator)->setAttribute(attribute, value);
}

/*! Keeps the list of solutions sorted according to some metric */
void BCIOnlinePlanner::updateSolutionList()
{
    boost::mutex::scoped_lock lock(mListAttributeMutex);


    std::list<GraspPlanningState *> for_deletion;
    std::list<GraspPlanningState*>::iterator it;
    //re-compute distance between current hand position and solutions.
    for ( it = mBestList.begin(); it != mBestList.end(); it++ )
    {

        double dist = (*it)->getEnergy();

        int reachable = 1;
        int unreachable = -1;
        int untested = 0;

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
        for(auto i=mBestList.begin();i!=mBestList.end();i++)
        {
            if((*delete_it)==(*i))
            {
                mBestList.erase(i);
                break;
            }

        }
    }

    mBestList.sort(GraspPlanningState::compareStatesDistances);
    //keep only best in list
    std::list<GraspPlanningState *>::iterator it2 = mBestList.begin();

    for(int i = 0; it2 != mBestList.end();)
    {
        if ((*it2)->getAttribute("testResult") <= 0)
        {
            break;
        }

        if(i >= SOLUTION_BUFFER_SIZE)
        {
            delete *it2;
            std::list<GraspPlanningState *>::iterator it3 = it2;
            ++it2;
            mBestList.erase(it3);
        }
        else
        {
            ++it2;
        }

        ++i;
    }

    while (mBestList.size() > 2*SOLUTION_BUFFER_SIZE)
    {
        delete mBestList.back();
        mBestList.pop_back();
    }

}

void 
BCIOnlinePlanner::mainLoop()
{
    static clock_t lastCheck = clock();
    clock_t time = clock();
    double secs = (float)(time - lastCheck) / CLOCKS_PER_SEC;

    if (secs < 0.2)
    {
        if (secs < 0.1)
        {
            //perform grasp planning all the time
            graspLoop();
            return;
        }

        lastCheck = time;

        //every 0.2 seconds, perform the management part:
        //set as a reference transform for the search the transform of the reference hand
        mCurrentState->setRefTran( mSeedHand->getTran(), false );
        //this is to ensure this (potentially) illegal state does not make it into the best list
        mCurrentState->setLegal(false);
        //re-set the legal search range along the approach direction, so we don't search pointlessly inside the object
        if ( mCurrentState->getVariable("dist"))
        {
            Body *obj = mCurrentState->getObject();
            double maxDist = 200;
            mObjectDistance = mSeedHand->getApproachDistance(obj,maxDist);
            if (mObjectDistance > maxDist) mObjectDistance = maxDist;
            mCurrentState->getPosition()->getVariable("dist")->setRange(-30, mObjectDistance);
            //make sure the current value is within range; otherwise simm ann can hang...
            mCurrentState->getPosition()->getVariable("dist")->setValue(mObjectDistance/2);
            mCurrentState->getPosition()->getVariable("dist")->setJump(0.33);
        }

        //is the planning part has produced new candidates, send them to the grasp tester
        std::list<GraspPlanningState*>::iterator it = mCandidateList.begin();
        while(it!=mCandidateList.end()) {
            //while there is space
            if ( mGraspTester->postCandidate(*it) )
            {
                DBGP("Candidate posted");
                it = mCandidateList.erase(it);
            }
            else
            {
                DBGP("Tester thread buffer is full");
                break;
            }
        }

        //retrieve solutions from the tester
        GraspPlanningState *s;
        boost::mutex::scoped_lock lock(mListAttributeMutex);
        while ( (s = mGraspTester->popSolution()) != NULL )
        {
            //hack - this is not ideal, but so far I don't have a better solution of how to keep track
            //of what hand is being used at what time
            s->changeHand( mSeedHand, true );//CHANGED! to mSolutionClone from mRefHand
            mBestList.push_back(s);
            if (mMarkSolutions)
            {
                mHand->getWorld()->getIVRoot()->addChild( s->getIVRoot() );
            }
        }

        updateSolutionList();

    }
    DBGP("On-line main loop done");
}



void BCIOnlinePlanner::graspLoop()
{
    //mTargetState is the position of the seedhand
    GraspPlanningState *input = mTargetState;

    //call simulated annealing
    SimAnn::Result r = mSimAnn->iterate(mCurrentState, mEnergyCalculator, input);
    mCurrentStep = mSimAnn->getCurrentStep();

    if ( r == SimAnn::JUMP )
    {
        assert(mCurrentState->isLegal());
        //we have a new state from the SimAnn
        if (mCurrentState->getEnergy() < 0 || mCurrentState->getEnergy() < mCurrentBest->getEnergy())
        {
            DBGP("New candidate");
            GraspPlanningState *insertState = new GraspPlanningState(mCurrentState);
            //make solution independent of reference hand position
            insertState->setPositionType(SPACE_COMPLETE, true);
            insertState->setRefTran( mCurrentState->getObject()->getTran(), true);
            insertState->setItNumber(mCurrentStep);
            if (insertState->getEnergy() < mCurrentBest->getEnergy())
            {
                mCurrentBest->copyFrom( insertState );
            }
            if (!addToListOfUniqueSolutions(insertState, &mCandidateList, 0.4))
            {
                DBGP("Similar to old candidate");
                delete insertState;
            }
            else
            {
                mCandidateList.sort(GraspPlanningState::compareStates);
                while (mCandidateList.size() > CANDIDATE_BUFFER_SIZE)
                {
                    delete mCandidateList.back();
                    mCandidateList.pop_back();
                }
            }
            DBGP("Added candidate");
        }
    }

    std::cout << "mCurrentStep: " << mCurrentStep << std::endl;
    if (mCurrentStep % 50 == 0)
    {
        std::cout << "BCI ONLINE PLANNER EMITTING UPDATE" << std::endl;
        emit update();
        //render(mHand);
    }
}

int BCIOnlinePlanner::getFCBufferSize()
{
    return mGraspTester->getNumCandidates();
}


//CHANGED! I added a special version of this to the OnLinePlanner so that it doesn't use the
// actual hand to show the results
void BCIOnlinePlanner::showGrasp(int i)
{
    const GraspPlanningState *s = getGrasp(i);
    s->execute(mSolutionClone);
}


bool BCIOnlinePlanner::addSolution(GraspPlanningState *s)
{
    bool addResult = addToListOfUniqueSolutions(s,&mBestList,0.2);
    mCurrentStep +=1;
    return addResult;
}


void BCIOnlinePlanner::render(Hand * h)
{
    if (mRenderType == RENDER_BEST)
    {
        if ( mBestList.empty() ) return;
        if ( mLastRenderState == mBestList.front() ) return;
        mLastRenderState = mBestList.front();
        mBestList.front()->execute(h);
    }
    else if (mRenderType == RENDER_LEGAL)
    {
        if (mRenderCount >= 20) {
            DBGP("Render: geom is " << mHand->getRenderGeometry() );
            mRenderCount = 0;
            if ( mCurrentState && mCurrentState->isLegal() ) mCurrentState->execute(h);
        } else mRenderCount++;
    }
    else if (mRenderType==RENDER_ALWAYS)
    {
        mCurrentState->execute(h);
    }
    else if ( mRenderType == RENDER_NEVER )
    {
        return;
    }
    else
    {
        DBGA("mRenderType is not valid!!!!!!")
    }
}
