#ifndef BCIONLINEPLANNER_2_H
#define BCIONLINEPLANNER_2_H

#include "include/EGPlanner/egPlanner.h"
#include "include/grasp.h"


class BCIOnlinePlanner2
{
   private:
    EGPlanner *candidateGenerator; //onLinePlanner
    //GraspCandidateTester tester; //graspTester
    //GraspReachabilityAnalyzer analyzer;
    std::vector<GraspPlanningState> cList;
    std::vector<GraspPlanningState> sList;

public:
    BCIOnlinePlanner2(Hand *h);
    void loadDBGrasps();
    void startReachabilityAnalysis();
    void stopReachabilityAnalysis();
    void startPlanning();
    void stopPlanning();
};


#endif // BCIONLINEPLANNER_2_H
