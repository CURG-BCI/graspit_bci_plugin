
#include "graspit_bci_plugin/graspit_bci_plugin.h"
#include "graspit_bci_plugin/bci_widget.h"

#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>


#include <QtGui>
#include <QFrame>
#include <QWidget>


namespace graspit_bci_plugin
{

    GraspitBCIPlugin::GraspitBCIPlugin()
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

//      QDialog *d = new QDialog();


//       d->show();
//       d->raise();
//       d->activateWindow();

        bciWidgetManager = new BCIWidgetManager();

//      QWidget *bciWidget = new QWidget();
//      renderArea = new SoQtRenderArea(bciWidget, " ",true);
//      SoSeparator * bciWorldViewRoot = new SoSeparator;
//      bciWorldViewRoot->setName("BCIWorldViewRoot");
//      SoMaterial * soMaterial = new SoMaterial;
//      SoTransformSeparator *lightSep = new SoTransformSeparator;
//      SoRotation *lightDir = new SoRotation;
//      SoLightModel * lightModel = new SoLightModel;

//      SoNode *ivRoot = graspItGUI->getIVmgr()->getViewer()->getSceneGraph();

//      soMaterial->diffuseColor.setValue(1,0,0);

//      lightDir->rotation.connectFrom(&graspItGUI->getIVmgr()->getViewer()->getCamera()->orientation);
//      lightSep->addChild(lightDir);
//      lightSep->addChild(graspItGUI->getIVmgr()->getViewer()->getHeadlight());

//      lightModel->model=SoLightModel::PHONG;

//      bciWorldViewRoot->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
//      bciWorldViewRoot->addChild(lightSep);
//      bciWorldViewRoot->addChild(lightModel);
//      bciWorldViewRoot->addChild(ivRoot);

//      renderArea->setSceneGraph(bciWorldViewRoot);
//      renderArea->setBackgroundColor(SbColor(1,1,1));
//      renderArea->scheduleRedraw();
//      renderArea->render();
//      renderArea->show();




//       bciWidget->show();
//       bciWidget->raise();
//       bciWidget->activateWindow();

//      QApplication app(argc, argv);

//      QGraphicsScene scene;
//      scene.setSceneRect(-300, -300, 600, 600);

//      scene.setItemIndexMethod(QGraphicsScene::NoIndex);

//      QGraphicsView view(&scene);

//      view.setRenderHint(QPainter::Antialiasing);
//      view.setBackgroundBrush(QPixmap(":/images/cheese.jpg"));

//      view.setCacheMode(QGraphicsView::CacheBackground);
//      view.setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
//      view.setDragMode(QGraphicsView::ScrollHandDrag);

//      view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
//  #if defined(Q_WS_S60) || defined(Q_WS_MAEMO_5) || defined(Q_WS_SIMULATOR)
//      view.showMaximized();
//  #else
//      view.resize(400, 300);
//      view.show();
//  #endif

//      QTimer timer;
//      QObject::connect(&timer, SIGNAL(timeout()), &scene, SLOT(advance()));

//      //graspItGUI->getIVmgr()


//      app.exec();


      //init ros
      ros::init(ros_argc, ros_argv, node_name.c_str());
      //ROS_INFO("Using node name %s", node_name.c_str());
      //clean up ros arguments
      for (int i = 0; i < argc; i++)
      {
        delete ros_argv[i];
      }
      delete ros_argv;

      //init node handles
      root_nh_ = new ros::NodeHandle("");

      ROS_INFO("ROS GraspIt node ready");
      return 0;



    }

    int GraspitBCIPlugin::mainLoop()
    {
      ros::spinOnce();
      return 0;
    }


}
