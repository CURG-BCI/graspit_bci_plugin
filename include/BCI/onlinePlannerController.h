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

            void drawGuides();
            void destroyGuides();
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

            void blockGraspAnalysis( bool block){analysisBlocked = block;}
            bool isAnalysisBlocked(){return analysisBlocked;}

            void incrementGraspIndex();
            void resetGraspIndex();
            void sortGrasps();
            unsigned int getNumGrasps();

            void showRobots(bool show);

            bool timedUpdateRunning;
            bool renderPending;

    protected:
        virtual void run();

    private:
            static OnlinePlannerController * onlinePlannerController;
            static QMutex createLock;

            BCIOnlinePlanner * currentPlanner;
            db_planner::SqlDatabaseManager * mDbMgr;
            GraspableBody * currentTarget;
            Hand * graspDemonstrationHand;

            unsigned int currentGraspIndex;
            bool sceneLocked;
            bool analysisBlocked;

            OnlinePlannerController(QObject *parent = 0);
            void initializeDbInterface();
            bool setAllowedPlanningCollisions();
            bool setPlannerTargets();
            void setCurrentTarget(GraspableBody * gb);
            void initializeTarget();

    signals:
            void render();

    private slots:
            // Perform any validation or processing that should update
            // the planner or it's visualizations periodically
            void plannerTimedUpdate();

    public slots:
            bool setPlannerToRunning();
            bool setPlannerToStopped();
            bool setPlannerToPaused();
            bool setPlannerToReady();

            void analyzeNextGrasp();
            void updateGraspReachability(int graspId, bool isReachable);

            void addToWorld(const QString model_filename, const QString object_name, const transf object_pose);
            void clearObjects();
            void targetRemoved();
            void emitRender(){if(!renderPending){ emit render(); renderPending = true;}}
    };

}
#endif // BCI_ONLINEPLANNERCONTROLLER_H

