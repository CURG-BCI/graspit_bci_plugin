#ifndef BCIONLINEPLANNER_2_H
#define BCIONLINEPLANNER_2_H


class BCIOnlinePlanner2
{
   private:
    GraspCandidateGenerator generator;//onLinePlanner
    GraspCandidateTester tester; //graspTester
    GraspReachabilityAnalyzer analyzer;
    CandidateList cList;
    SolutionList sList;

public:

    void loadDBGrasps(sList);
    void startReachabilityAnalysis();
    void stopReachabilityAnalysis();
    void startPlanning();
    void stopPlanning();
};


#endif // BCIONLINEPLANNER_2_H
