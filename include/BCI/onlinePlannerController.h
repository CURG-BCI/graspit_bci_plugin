#ifndef BCI_ONLINEPLANNERCONTROLLER_H
#define BCI_ONLINEPLANNERCONTROLLER_H

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

    class OnlinePlannerController : public QThread
    {
        Q_OBJECT

        public:

            static OnlinePlannerController * getInstance();

            bool analyzeApproachDir();
            bool hasRecognizedObjects();

            void rotateHandLat();
            void rotateHandLong();

            void alignHand();

            GraspableBody* getCurrentTarget();
            GraspableBody * incrementCurrentTarget();

            Hand * getHand();
            Hand * getSeedHand();
            Hand * getSolutionHand();

            const GraspPlanningState * getGrasp(int index);
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
            void sortGrasps();
            unsigned int getNumGrasps();

            void drawGuides();
            void destroyGuides();

            void showRobots(bool show);

            void showSeedHand(bool show);
            void showMHand(bool show);
            void showSolutionHand(bool show);


            bool timedUpdateRunning;
            bool renderPending;

    protected:
        virtual void run();

    private:
            static OnlinePlannerController * onlinePlannerController;
            static QMutex createLock;

            BCIOnlinePlanner * mPlanner;
            ReachabilityAnalyzer *mReachabilityAnalyzer;

            db_planner::SqlDatabaseManager * mDbMgr;
            GraspableBody * currentTarget;
            Hand * graspDemonstrationHand;

            unsigned int currentGraspIndex;
            bool sceneLocked;

            OnlinePlannerController(QObject *parent = 0);
            void initializeDbInterface();
            void initializeTarget();

            bool setAllowedPlanningCollisions();
            bool setPlannerTargets();
            void setCurrentTarget(GraspableBody * gb);

            void analyzeNextGraspReachability();


            bool plannerCanBeSetToReady();
            bool plannerCanBeSetToRunning();

    signals:
            void render();

    private slots:
            // Perform any validation or processing that should update
            // the planner or it's visualizations periodically
            void plannerTimedUpdate();

    public slots:
            void setPlannerToRunning();
            bool setPlannerToStopped();
            bool setPlannerToPaused();
            void setPlannerToReady();

            //! analyze reachability of grasps from database without planning new grasps
            bool startGraspReachabilityAnalysis();
            //! analyze reachability of grasps from database and plan new grasps
            bool stopGraspReachabilityAnalysis();

            void analyzeNextGraspReachabilityCallback(int graspId, bool isReachable);

            void addToWorld(const QString model_filename, const QString object_name, const transf object_pose);
            void clearObjects();
            void targetRemoved();
            void emitRender();
    };

}
#endif // BCI_ONLINEPLANNERCONTROLLER_H

