#ifndef PLANNER_TOOLS_H_
#define PLANNER_TOOLS_H_
#include <vector>

#include "include/DBase/graspit_db_model.h"

#include "include/DBase/graspit_db_grasp.h"

class EGPlanner;
class Body;
class GraspPlanningState;


namespace bci_experiment{

    namespace planner_tools{

        //! Import grasps from a database to an egPlanner
        void importGraspsFromDBMgr(Hand* mHand, GraspableBody * target, std::vector<GraspPlanningState*>& graspList,
                                    db_planner::DatabaseManager * mDbMgr);

        static QString getHandDBName(Hand* h);

    }


}
#endif

