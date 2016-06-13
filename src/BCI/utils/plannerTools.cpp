#include "include/world.h"
#include "include/robot.h"
#include "include/EGPlanner/searchState.h"
#include "include/EGPlanner/egPlanner.h"
#include "include/debug.h"

#include "src/DBase/DBPlanner/db_manager.h"
#include "src/DBase/graspit_db_grasp.h"

#include "BCI/utils/worldElementTools.h"
#include "BCI/utils/plannerTools.h"


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::world_element_tools::getOrAddExperimentTable;

namespace bci_experiment
{
    namespace planner_tools
    {

        void importGraspsFromDBMgr(Hand*mHand, GraspableBody * target, std::vector<GraspPlanningState*>& graspList, db_planner::DatabaseManager * mDbMgr)
        {
            db_planner::Model *m=new db_planner::Model;

            m->SetModelName(target->getName().remove(".xml").remove(QRegExp("_[0-9]{1,2}")).toStdString());

            std::vector<db_planner::Grasp*> grasps;
            mDbMgr->GetGrasps(*m, getHandDBName(mHand).toStdString(), &grasps);
            mHand->saveState();

            if(grasps.empty())
            {
              DBGA("No Grasps Found \n");
              return;
            }

            // Load the grasps into the grasp planner list.
            unsigned int numGrasps = std::min<unsigned int>(grasps.size(), 10);
            for (unsigned int gNum = 0; gNum < numGrasps; ++gNum)
            {
                GraspPlanningState *s = new GraspPlanningState(static_cast<GraspitDBGrasp *>
                                       (grasps[gNum])->getFinalGraspPlanningState());

                s->setObject(target);
                s->setRefTran(target->getTran());
                float testResult = 0;
                s->addAttribute("graspId", gNum);
                s->addAttribute("testResult", testResult);
                s->addAttribute("testTime", 0);
                graspList.push_back(s);
                std::cout << "adding grasp to planner solutions" << std::endl;
            }

            //needed to return hand to aligned with object, since it was used to testGraspCollisions
            mHand->restoreState();

        }

        //! get the hand name that is in the CGDB
        QString getHandDBName(Hand* h)
        {
            QString handName = h->getName();
            QString hand_db_name;
            if(handName.contains("arrett"))
            {
              std::cout << "GETTING GRASPS FOR NEW_BARRETT_RUBBER" << std::endl;
                return QString("NEW_BARRETT_RUBBER");
            }
            else if(handName.split(' ')[0] == QString("MicoGripper"))
            {
              std::cout << "GETTING GRASPS FOR MICO_GRIPPER" << std::endl;
                return QString("MICO_GRIPPER");
            }
            else {
                std::cout << "Wrong hand name detected: " << handName.toStdString().c_str() <<
                    ".  Acceptable GRASPIT_hand_names are: Barrett, HumanHand20DOF, Pr2Gripper" << std::endl;
                hand_db_name = QString::null;
            }
            return hand_db_name;
        }

    }

}
