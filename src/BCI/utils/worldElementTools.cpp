
#include "BCI/utils/worldElementTools.h"
#include "include/debug.h"


namespace bci_experiment{
namespace world_element_tools{


Body * getOrAddExperimentTable()
{
    QString bodyName("experiment_table");
    Body * tableBody= getObjectByName(bodyName);
    if(tableBody)
    {
        return tableBody;
    }
    return addToWorld("models/objects/","Obstacle", bodyName+".xml");
}


GraspableBody * getNextGraspableBody(GraspableBody * b)
{
    GraspableBody * nextBody = NULL;
    World * w = getWorld();
    if(b)
    {
        int bodyIndex = getGraspableBodyIndex(b);
        if(bodyIndex == -1)
        {
            std::cout << "getNextGraspableBody -- ERROR -- attempted to get next of unregistered body -- \n";
            return NULL;
        }
        int nextBodyIndex = (bodyIndex + 1) % w->getNumGB();
        nextBody = w->getGB(nextBodyIndex);
    }
    else if(w->getNumGB() > 0)
    {
        nextBody = getWorld()->getGB(0);
    }


    return nextBody;
}



int getGraspableBodyIndex(Body * b)
{
    World * w = getWorld();
    int bodyIndex = -1;
    while(bodyIndex < w->getNumGB() - 1)
    {
        ++bodyIndex;
        if(w->getGB(bodyIndex) == b)
            break;
    }
    return bodyIndex;
}

Body * getObjectByName(const QString & objectName)
{
    World * w = getWorld();
    Body * b = NULL;
    for(int i = 0; i < w->getNumBodies(); ++i)
    {
        if(w->getBody(i)->getName() == objectName)
            b = w->getBody(i);
    }
    return b;
}


World * getWorld()
{
    return graspitCore->getWorld();
}


//! Brief helper to add an object to the world from relative directory, model type, and model filename.
Body * addToWorld(const QString & relativeModelDir, const QString & modelType, const QString & modelFilename)
{
    QString bodyFile = QString(getenv("GRASPIT")) + "/" +  relativeModelDir + modelFilename;
    std::cout << "body string: "<< bodyFile.toStdString() << std::endl;
    return graspitCore->getWorld()->importBody(modelType, bodyFile);
}


//! If the named body doesn't exist, add it to the world
Body * addBodyIfUnique(const QString & bodyName)
{
    Body * newBody = getObjectByName(bodyName);
    if(newBody)
        return newBody;
    newBody = addToWorld("models/objects/","Body", bodyName+".xml");

    return newBody;
}

//! Aligns the hand to an object
void alignHandToObject(Hand * hand, Body * targetBody, double distance)
{
    // Get approach transform so that we can rotate it in to the object
    transf approachTran = hand->getApproachTran()*hand->getTran();


    //If the distance is legal, try to set the hand to the desired distance,
    //but respect collisions.
    if(distance > 0)
        hand->approachToContact((approachTran.translation() - targetBody->getTran().translation()).len() - distance, true);
}

}
}
