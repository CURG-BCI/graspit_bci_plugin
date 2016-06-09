
#include "graspit_bci_plugin/graspit_bci_plugin.h"

#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <bciService.h>
#include <bciControlWindow.h>
#include "ui/mainWindow.h"

#include <QtGui>
#include <QFrame>
#include <QWidget>


namespace graspit_bci_plugin
{

GraspitBCIPlugin::GraspitBCIPlugin():
    isInited(false)
{
    ROS_INFO("BCI PLUGIN STARTING");
}

GraspitBCIPlugin::~GraspitBCIPlugin()
{
    ROS_INFO("ROS GraspIt node stopping");
    ros::shutdown();
}


int GraspitBCIPlugin::init(int argc, char **argv)
{

    server_thread = new QThread;
    BCIControlServer *worker = new BCIControlServer();
    worker->moveToThread(server_thread);
    QObject::connect(server_thread, SIGNAL(started()), worker, SLOT(process()));
    server_thread->start();


    qRegisterMetaType<transf>("transf");
    //copy the arguments somewhere else so we can pass them to ROS
    int ros_argc = argc;
    char** ros_argv = new char*[argc];
    for (int i = 0; i < argc; i++)
    {
        ros_argv[i] = new char[strlen(argv[i])];
        strcpy(ros_argv[i], argv[i]);
    }
    //see if a node name was requested
    std::string node_name("ros_graspit_interface");
    for (int i = 0; i < argc - 1; i++)
    {
        //std::cerr << argv[i] << "\n";
        if (!strcmp(argv[i], "_name"))
        {
            node_name = argv[i + 1];
        }
    }

    ros::init(ros_argc, ros_argv, node_name.c_str());
    root_nh_ = new ros::NodeHandle("");

    for (int i = 0; i < argc; i++)
    {
        delete ros_argv[i];
    }
    delete ros_argv;



    //this should go away, just a hack
    QString worldfile = QString(getenv("GRASPIT")) + QString("/worlds/allVision2fromjon.xml");
    std::cout<<worldfile.toStdString().c_str()<<std::endl;
    graspItGUI->getIVmgr()->getWorld()->load(worldfile.toStdString().c_str());




    ROS_INFO("Finished initing BCI Plugin");

    return 0;
}

int GraspitBCIPlugin::mainLoop()
{
    if(!isInited)
    {
        isInited = true;
        BCIControlWindow *bciControlWindow= new BCIControlWindow();
        BCIService::getInstance()->init(bciControlWindow);
        bciControlWindow->show();
    }
    ros::spinOnce();
    return 0;
}


}
