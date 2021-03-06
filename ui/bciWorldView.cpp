#include "bciWorldView.h"
#include "ui_bciWorldView.h"

BCIWorldView::BCIWorldView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::BCIWorldView)
{
    ui->setupUi(this);

    renderArea = new SoQtRenderArea(this, " ",true);
    SoSeparator * bciWorldViewRoot = new SoSeparator;
    bciWorldViewRoot->setName("BCIWorldViewRoot");
    SoMaterial * soMaterial = new SoMaterial;
    SoTransformSeparator *lightSep = new SoTransformSeparator;
    SoRotation *lightDir = new SoRotation;
    SoLightModel * lightModel = new SoLightModel;

    SoNode *ivRoot = graspitCore->getIVmgr()->getViewer()->getSceneGraph();

    soMaterial->diffuseColor.setValue(1,0,0);

    lightDir->rotation.connectFrom(&graspitCore->getIVmgr()->getViewer()->getCamera()->orientation);
    lightSep->addChild(lightDir);
    lightSep->addChild(graspitCore->getIVmgr()->getViewer()->getHeadlight());

    lightModel->model=SoLightModel::PHONG;

    bciWorldViewRoot->addChild(graspitCore->getIVmgr()->getViewer()->getCamera());
    bciWorldViewRoot->addChild(lightSep);
    bciWorldViewRoot->addChild(lightModel);
    bciWorldViewRoot->addChild(ivRoot);

    renderArea->setSceneGraph(bciWorldViewRoot);
    renderArea->setBackgroundColor(SbColor(1,1,1));
    renderArea->scheduleRedraw();
    renderArea->render();
    renderArea->show();





}

void BCIWorldView::redraw()
{
    renderArea->render();
}

BCIWorldView::~BCIWorldView()
{
    delete ui;
}

void BCIWorldView::drawGuides()
{

}
