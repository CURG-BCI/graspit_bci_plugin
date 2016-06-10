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
        bool testPreGraspCollisions(Hand * h, float pregrasp_dist)
        {
          h->autoGrasp(false, -2.0, true);
          h->approachToContact(pregrasp_dist, false);
          return (getNumHandCollisions(h));
        }


        bool testGraspCollisions(Hand * h, const GraspPlanningState * s)
        {
          bool result = false;
          std::vector<bool> currentCollisionState;
          resetHandCollisions(h, true, currentCollisionState);
          s->execute(h);
          World * w = getWorld();
          w->toggleCollisions(false, h, s->getObject());
          if(getNumHandCollisions(h))
            result = true;
          if(testPreGraspCollisions(h, -50.0))
            result = true;

          setCollisionState(h, currentCollisionState);
          return result;
        }


        int getNumHandCollisions(Hand * h)
        {
//          CollisionReport colReport;
//          std::vector<Body *> body_list;
//          h->getBodyList(&body_list);
//          getWorld()->getCollisionReport(&colReport, &body_list);
//          return colReport.size();
          return 0;
        }

        bool setCollisionState(Hand * h, std::vector<bool> & collisionStatus)
        {
          World * w = getWorld();
          if(collisionStatus.size() != w->getNumGB() + 1)
            return false;

          Body * experiment_table = getOrAddExperimentTable();
          w->toggleCollisions(collisionStatus[0], h, experiment_table);
          for (int i = 0; i < w->getNumGB(); ++i)
          {
            w->toggleCollisions(collisionStatus[i+1], h, w->getGB(i));
          }
          return true;
        }


        void resetHandCollisions(Hand * h, bool setting,
                                 std::vector<bool> & collisionStatus)
        {
          World * w = getWorld();

          Body * experiment_table = getOrAddExperimentTable();
          collisionStatus.push_back(!w->collisionsAreOff(h, experiment_table));
          w->toggleCollisions(setting, h, experiment_table);

          for (int i = 0; i < w->getNumGB(); ++i)
          {
            collisionStatus.push_back(!w->collisionsAreOff(h, w->getGB(i)));
            w->toggleCollisions(setting, h, w->getGB(i));
          }
        }





        void importGraspsFromDBMgr(Hand*mHand, std::vector<GraspPlanningState*>& graspList, db_planner::DatabaseManager * mDbMgr)
        {
            db_planner::Model *m=new db_planner::Model;

            m->SetModelName(mHand->getGrasp()->getObject()->getName().remove(".xml").remove(QRegExp("_[0-9]{1,2}")).toStdString());

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

                s->setObject(mHand->getGrasp()->getObject());
                s->setRefTran(mHand->getGrasp()->getObject()->getTran());
                float testResult = -2*bci_experiment::planner_tools::testGraspCollisions(mHand, s);
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
