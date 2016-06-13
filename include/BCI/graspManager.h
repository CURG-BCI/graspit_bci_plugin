#ifndef GRASPMANAGER_H
#define GRASPMANAGER_H

#include <QObject>

#include "include/robot.h"
#include "include/grasp.h"
#include "include/graspitGUI.h"
#include "include/ivmgr.h"

#include "src/DBase/DBPlanner/sql_database_manager.h"
#include "src/DBase/DBPlanner/db_manager.h"
#include "src/DBase/DBPlanner/database.h"

#include "BCI/utils/uiTools.h"
#include "BCI/utils/plannerTools.h"
#include "BCI/utils/worldElementTools.h"
#include "BCI/reachabilityAnalyzer.h"

class GraspableBody;
class GraspPlanningState;

namespace bci_experiment{

    class GraspManager : public QThread
    {
        Q_OBJECT

        public:

            static GraspManager * getInstance();

            bool hasRecognizedObjects();

            GraspableBody* getCurrentTarget();
            GraspableBody * incrementCurrentTarget();

            Hand * getHand();

            GraspPlanningState * getGrasp(int index);
            const GraspPlanningState * getCurrentGrasp();
            const GraspPlanningState * getNextGrasp();

            bool stopTimedUpdate();
            bool startTimedUpdate();
            bool toggleTimedUpdate();

            void setSceneLocked(bool locked){sceneLocked = locked;}
            bool isSceneLocked(){return sceneLocked;}

            void incrementGraspIndex();
            void decrementGraspIndex();
            void resetGraspIndex();
            unsigned int getNumGrasps();

            void showRobots(bool show);

            void showMHand(bool show);

            void clearGrasps();
            void addGrasp(GraspPlanningState *gps);


            bool timedUpdateRunning;
            bool renderPending;

            void updateSolutionList();

    protected:
            virtual void run();

    private:
            static GraspManager * graspManager;
            static QMutex createLock;

            ReachabilityAnalyzer *mReachabilityAnalyzer;

            db_planner::SqlDatabaseManager * mDbMgr;
            unsigned int currentTargetIndex;
            Hand * mHand;

            unsigned int currentGraspIndex;
            bool sceneLocked;

            GraspManager(QObject *parent = 0);
            void initializeDbInterface();

            void analyzeNextGraspReachability();

            std::vector<GraspPlanningState*> mGraspList;

    signals:
            void render();

    public slots:
            void getGraspsFromDB();

            //! analyze reachability of grasps from database without planning new grasps
            bool startGraspReachabilityAnalysis();
            //! analyze reachability of grasps from database and plan new grasps
            bool stopGraspReachabilityAnalysis();

            void analyzeNextGraspReachabilityCallback(int graspId, bool isReachable);

            void addToWorld(const QString model_filename, const QString object_name, const transf object_pose);
            void clearObjects();
            void emitRender();
    };

}
#endif

