#include "BCI/bciService.h"
#include "BCI/BCIStateMachine.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/nodes/SoLight.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoCallback.h>
#include <Inventor/elements/SoCacheElement.h>
#include <Inventor/nodes/SoCallback.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoRotationXYZ.h>

#include <Inventor/actions/SoGLRenderAction.h>
#include <QtOpenGL/QGLWidget>
#include <QDialog>
#include <QtGui>

#include "BCI/utils/BCILogger.h"


BCIService * BCIService::bciServiceInstance = NULL;

BCIService* BCIService::getInstance()
{
    if(!bciServiceInstance)
    {
        bciServiceInstance = new BCIService();
    }

    return bciServiceInstance;
}

BCIService::BCIService():QObject()
{

}

void BCIService::init(BCIControlWindow *bciControlWindow)
{
    ROS_INFO("Initing BCI Service");
    bciRenderArea = bciControlWindow->bciWorldView->renderArea;

    QPushButton * slowButton = new QPushButton("Cycle Choice - (Flex Medium)");
    QPushButton * fastButton = new QPushButton("Select Choice - (Flex Hard)");

    slowButton->setDefault(true);
    fastButton->setDefault(true);

    QDialogButtonBox *cursorControlBox = new QDialogButtonBox(Qt::Vertical);
    cursorControlBox->setCaption(QString("Cursor Control Box"));

    cursorControlBox->addButton(slowButton, QDialogButtonBox::ActionRole);
    cursorControlBox->addButton(fastButton, QDialogButtonBox::ActionRole);
    cursorControlBox->setWindowFlags(Qt::WindowStaysOnTopHint);
    cursorControlBox->resize(QSize(210, 90));
    cursorControlBox->show();

    QObject::connect(slowButton, SIGNAL(clicked()), this, SLOT(updateControlSceneState1()));
    QObject::connect(fastButton, SIGNAL(clicked()), this, SLOT(updateControlSceneState2()));

    endOfExperimentFormat = new QFormLayout();

    successFailure = new QComboBox();
    successFailure->insertItem("Success");
    successFailure->insertItem("Failure");

    finalComments = new QTextEdit();

    finishedButton = new QPushButton("FINISHED");


    endOfExperimentFormat->addRow(tr("&Success or failure: "), successFailure);
    endOfExperimentFormat->addRow(tr("&Comments: "), finalComments);
    endOfExperimentFormat->addRow(tr(""), finishedButton);

    endOfExperiment = new QGroupBox();
    endOfExperiment->setLayout(endOfExperimentFormat);



    QObject::connect(finishedButton, SIGNAL(clicked()), this, SLOT(onFinishedFinalLog()));

    endOfExperiment->show();

    timer = new QTimer;
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateControlScene()));
    timer->start(1000 / 30);

    QObject::connect(this, SIGNAL(goToStateLow()), this, SLOT(updateControlSceneState0()));
    QObject::connect(this, SIGNAL(goToStateMedium()), this, SLOT(updateControlSceneState1()));
    QObject::connect(this, SIGNAL(goToStateHigh()), this, SLOT(updateControlSceneState2()));

    SoAnnotation *hudSeparator = new SoAnnotation;
    hudSeparator->renderCaching = SoSeparator::OFF;
    hudSeparator->setName("hud");
    graspitCore->getWorld()->getIVRoot()->addChild(hudSeparator);
    pcam = new SoOrthographicCamera;
    pcam->position = SbVec3f(0, 0, 10);
    pcam->nearDistance = 0.1;
    pcam->farDistance = 11;

    hudSeparator->addChild(pcam);
    SoLightModel * hudLightModel = new SoLightModel;
    hudLightModel->model=SoLightModel::BASE_COLOR;
    hudSeparator->addChild(hudLightModel);
    SoCallback * disableZTestNode = new SoCallback();
    disableZTestNode->setCallback(ui_tools::disableZCulling);
    hudSeparator->addChild(disableZTestNode);
    csm = new ControllerSceneManager(hudSeparator);

    BCIStateMachine *bciStateMachine = new BCIStateMachine(bciControlWindow,this);
    connect(this, SIGNAL(plannerUpdated()), bciControlWindow, SLOT(redraw()));
    connect(GraspManager::getInstance(), SIGNAL(render()), bciControlWindow, SLOT(redraw()));
    bciStateMachine->start();

    ROS_INFO("Finished initing BCI Service");
}

void BCIService::updateControlSceneState0()
{
    std::cout << "csm->setState(0)" << std::endl;
    csm->setState(0);
}
void BCIService::updateControlSceneState1()
{
    std::cout << "csm->setState(1)" << std::endl;
    csm->setState(1);
}
void BCIService::updateControlSceneState2()
{
    std::cout << "csm->setState(2)" << std::endl;
    csm->setState(2);
}

void BCIService::updateControlScene()
{
    csm->update();
    bciRenderArea->setViewportRegion(pcam->getViewportBounds(bciRenderArea->getViewportRegion()));
}

void BCIService::onFinishedFinalLog()
{
    endOfExperiment->close();
    QString result = QString(successFailure->currentText());
    finalComments->selectAll();
    QString comments = QString(finalComments->selectedText());
    BCILogger::getInstance()->writeExperimentSettings("Result", result);
    BCILogger::getInstance()->writeExperimentSettings("Final comments", comments);
}
void BCIService::setRos(ros::NodeHandle *_nh)
{
    n = _nh;
}
