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
#include <QPushButton>
#include <QDialog>
#include <QtGui>


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

    //QPushButton * spinButton = new QPushButton("Spin");
    QPushButton * slowButton = new QPushButton("Flex Medium (Next)");
    QPushButton * fastButton = new QPushButton("Flex Hard (Select)");

    //spinButton->setDefault(true);
    slowButton->setDefault(true);
    fastButton->setDefault(true);

    QDialogButtonBox *cursorControlBox = new QDialogButtonBox(Qt::Vertical);
    cursorControlBox->setCaption(QString("Cursor Control Box"));

    //cursorControlBox->addButton(spinButton, QDialogButtonBox::ActionRole);
    cursorControlBox->addButton(slowButton, QDialogButtonBox::ActionRole);
    cursorControlBox->addButton(fastButton, QDialogButtonBox::ActionRole);
    cursorControlBox->setWindowFlags(Qt::WindowStaysOnTopHint);
    cursorControlBox->resize(QSize(200,100));
    cursorControlBox->show();

    //QObject::connect(spinButton, SIGNAL(clicked()), this, SLOT(updateControlSceneState0()));
    QObject::connect(slowButton, SIGNAL(clicked()), this, SLOT(updateControlSceneState1()));
    QObject::connect(fastButton, SIGNAL(clicked()), this, SLOT(updateControlSceneState2()));

    timer = new QTimer;
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateControlScene()));
    timer->start(1000 / 30);

    QObject::connect(this, SIGNAL(goToStateLow()), this, SLOT(updateControlSceneState0()));
    QObject::connect(this, SIGNAL(goToStateMedium()), this, SLOT(updateControlSceneState1()));
    QObject::connect(this, SIGNAL(goToStateHigh()), this, SLOT(updateControlSceneState2()));

    SoAnnotation *hudSeparator = new SoAnnotation;
    hudSeparator->renderCaching = SoSeparator::OFF;
    hudSeparator->setName("hud");
    graspItGUI->getIVmgr()->getWorld()->getIVRoot()->addChild(hudSeparator);
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
    connect(OnlinePlannerController::getInstance(), SIGNAL(render()), bciControlWindow, SLOT(redraw()));
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



