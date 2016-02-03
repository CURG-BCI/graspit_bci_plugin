
#include "BCIOnlinePlanner_2.h"
#include "include/EGPlanner/searchState.h"
#include "include/EGPlanner/simAnnPlanner.h"
#include "include/EGPlanner/onLinePlanner.h"

/**
class BCIOnlinePlanner2
{
   private:
    EGPlanner *candidateGenerator; //onLinePlanner
    //GraspCandidateTester tester; //graspTester
    GraspReachabilityAnalyzer analyzer;
    std::vector<GraspPlanningState> cList;
    std::vector<GraspPlanningState> sList;

public:

    void loadDBGrasps();
    void startReachabilityAnalysis();
    void stopReachabilityAnalysis();
    void startPlanning();
    void stopPlanning();
};
**/


BCIOnlinePlanner2::BCIOnlinePlanner2(Hand *h)
{

    candidateGenerator = new SimAnnPlanner(h);
}

void BCIOnlinePlanner2::loadDBGrasps()
{
  //Todo
}

void BCIOnlinePlanner2::startReachabilityAnalysis()
{
  //Todo
}

void BCIOnlinePlanner2::stopReachabilityAnalysis()
{
 //Todo
}

void BCIOnlinePlanner2::startPlanning()
{
    candidateGenerator->startPlanner();
}

void BCIOnlinePlanner2::stopPlanning()
{
    candidateGenerator->stopPlanner();
}

