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





        void importGraspsFromDBMgr( BCIOnlinePlanner * mPlanner, db_planner::DatabaseManager * mDbMgr)
        {
            Hand*mHand = mPlanner->getSeedHand();

            db_planner::Model *m=new db_planner::Model;

            m->SetModelName(mHand->getGrasp()->getObject()->getName().remove(".xml").toStdString());

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
                mPlanner->addSolution(s);
                std::cout << "adding grasp to planner solutions" << std::endl;
            }

            //reorders the solutions we have found.
            mPlanner->updateSolutionList();

            //needed to return hand to aligned with object, since it was used to testGraspCollisions
            mHand->restoreState();

        }

        BCIOnlinePlanner * createDefaultPlanner(){

             World * w = getWorld();
             if(!w->getCurrentHand())
             {
                 DBGA("plannerTools::createDefaultPlanner::No current Hand!");
                 return NULL;
             }

             GraspPlanningState *mHandObjectState = new GraspPlanningState(w->getCurrentHand());
             mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
             mHandObjectState->setObject(w->getGB(0));
             mHandObjectState->setRefTran(w->getGB(0)->getTran());
             mHandObjectState->reset();

             BCIOnlinePlanner * op = new BCIOnlinePlanner(w->getCurrentHand());
             op->setContactType(CONTACT_PRESET);
             op->setEnergyType(ENERGY_CONTACT_QUALITY);
             op->setMaxSteps(2000);
             op->setModelState(mHandObjectState);
             op->showSolutionClone(true);
             op->resetPlanner();

            return op;
        }

        //! get the hand name that is in the CGDB
        QString getHandDBName(Hand* h)
        {
            QString handName = h->getName();
            QString hand_db_name;
            if(handName.contains("arrett"))
            {
                return QString("NEW_BARRETT_RUBBER");
            }
            else if(handName.split(' ')[0] == QString("MicoGripper"))
            {
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
