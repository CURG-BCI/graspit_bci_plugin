
#ifndef _ROS_GRASPIT_GraspitBCIPlugin_INTERFACE_H_
#define _ROS_GRASPIT_GraspitBCIPlugin_INTERFACE_H_

#include <map>

//GraspIt! includes
#include <include/plugin.h>
#include "BCIControlServer.h"

#include <QWidget>

#include<Inventor/Qt/SoQtRenderArea.h>
#include<Inventor/nodes/SoSeparator.h>
#include<Inventor/nodes/SoMaterial.h>
#include<Inventor/nodes/SoTransformSeparator.h>

#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include<Inventor/nodes/SoLightModel.h>


namespace db_planner
{
    class DatabaseManager;
}

class GraspitDBModel;
class Body;
class transf;
class GraspableBody;

#include <ros/ros.h>

#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/GraspPlanning.h>

namespace graspit_bci_plugin
{

//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.

  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.

  In particular, note that this class uses the mainLoop() function to perform the ROS even management calls.
*/
class GraspitBCIPlugin : public Plugin
{

public:
    //! Node handle in the root namespace
    ros::NodeHandle *root_nh_;


    QThread* server_thread;

    bool isInited;

    //! Inits ROS, but (for now) without passing any arguments
    GraspitBCIPlugin();
    //! Deletes the node handle and the db manager
    ~GraspitBCIPlugin();
    //! Creates the node handles, advertises services, connects to the database
    virtual int init(int argc, char **argv);
    //! Simply calls ros::spinOnce() to process the ROS event loop
    virtual int mainLoop();
};

}

#endif //_ROS_GRASPIT_GraspitBCIPlugin_INTERFACE_H_
