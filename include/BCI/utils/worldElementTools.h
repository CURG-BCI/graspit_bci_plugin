#ifndef WORLD_ELEMENT_TOOLS_H_
#define WORLD_ELEMENT_TOOLS_H_

#include "include/world.h"
#include "include/body.h"
#include "include/ivmgr.h"
#include "include/robot.h"
#include "include/graspitGUI.h"
#include "include/grasp.h"

namespace bci_experiment{
namespace world_element_tools{

World * getWorld();


//! Gets first object that matches the given name.
Body * getObjectByName(const QString & objectName);

//! Brief helper to add an object to the world from relative directory, model type, and model filename.
Body * addToWorld(const QString & relativeModelDir,
                  const QString & modelType,
                  const QString & modelFilename);

//! If the named body doesn't exist, add it to the world
Body * addBodyIfUnique(const QString & bodyName);


//! Get the next body in the graspable body list
GraspableBody * getNextGraspableBody(GraspableBody * b);

//! Find the index of a given body in the world's graspable body vector
int getGraspableBodyIndex(Body * b);


//! Add an experiment table to world or get a pointer to an existing one
Body * getOrAddExperimentTable();

}

}


#endif
