#ifndef BCISERVICE_H
#define BCISERVICE_H

#include <QObject>
#include <QTimer>
#include <QMutex>
#include <vector>

#include <QPushButton>

#include <QFormLayout>
#include <QComboBox>
#include <QTextEdit>
#include <QGroupBox>

#include "BCI/worldController.h"
#include "BCI/controller_scene/controller_scene_mgr.h"

#include <Inventor/nodes/SoOrthographicCamera.h>
#include<Inventor/Qt/SoQtRenderArea.h>

#ifndef Q_MOC_RUN
#include "ros/ros.h"
#endif

class QImage;
class QString;

class GraspableBody;
class GraspPlanningState;
class BCIControlWindow;
class ControllerSceneManager;


using namespace bci_experiment;

class BCIService:public QObject
{

    Q_OBJECT

public:
    ~BCIService(){}

    ControllerSceneManager * csm;
    QTimer *timer;

    //these are the change state signals used by the bci state machine states
    void emitGoToNextState1(){emit goToNextState1();}
    void emitGoToNextState2(){emit goToNextState2();}
    void emitGoToPreviousState(){emit goToPreviousState();}
    void emitFinishedRecognition(){emit finishedRecognition();}
    void emitFinishedCollectingUserInfo(){emit finishedCollectingUserInfo();}
    void emitExec(){emit exec();}
    void emitNext(){emit next();}

    //these are emitted by the bci emg device
    void emitGoToStateLow(){emit goToStateLow();}
    void emitGoToStateMedium(){emit goToStateMedium();}
    void emitGoToStateHigh(){emit goToStateHigh();}

    void emitAnalyzeApproachDir(GraspPlanningState * gs){emit analyzeApproachDir(gs);}

    static BCIService* getInstance();

    void init(BCIControlWindow *bciControlWindow);
    SoQtRenderArea *bciRenderArea;

    void setRos(ros::NodeHandle *_nh);

public slots:
    //called when active planner is updated
    void onPlannerUpdated(){emit plannerUpdated();}
    void emitProcessWorldPlanner(int i){emit processWorldPlanner(i);}

    void updateControlSceneState0();
    void updateControlSceneState1();
    void updateControlSceneState2();

    void updateExperimentBlock();
    void updateExperimentObject();

    void updateControlScene();
    void onFinishedFinalLog();


signals:

    // state machine transition signals
    void goToNextState1();
    void goToNextState2();
    void goToPreviousState();
    void exec();
    void next();
    void rotLong();
    void rotLat();
    void finishedCollectingUserInfo();
    void finishedRecognition();

    void plannerUpdated();

    void runObjectRecognitionSignal();
    void getCameraOriginSignal();
    void checkGraspReachabilitySignal();

    //! Signal that planner grasps should be processed or sent out for execution
    void processWorldPlanner(int solutionIndex);

    void analyzeApproachDir(GraspPlanningState * gps);

    void goToStateLow();
    void goToStateMedium();
    void goToStateHigh();

private:
        //singleton pattern, single static instance of the class
        static BCIService * bciServiceInstance;

        //this is singleton, so constructor must be private.
        BCIService();

        SoOrthographicCamera * pcam;

        QTextEdit * finalComments;
        QFormLayout * endOfExperimentFormat;
        QComboBox * successFailure;
        QPushButton * finishedButton;
        QGroupBox * endOfExperiment;

        ros::NodeHandle *n;

};

#endif // BCISERVICE_H
