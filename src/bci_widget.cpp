
#ifndef BCIWORLDVIEW_H
#define BCIWORLDVIEW_H

#include "graspit_bci_plugin/bci_widget.h"

BCIWidgetManager::BCIWidgetManager()
{
    QWidget *bciWidget = new QWidget();

    renderArea = new SoQtRenderArea(bciWidget, " ",true);
    SoSeparator * bciWorldViewRoot = new SoSeparator;
    bciWorldViewRoot->setName("BCIWorldViewRoot");
    SoMaterial * soMaterial = new SoMaterial;
    SoTransformSeparator *lightSep = new SoTransformSeparator;
    SoRotation *lightDir = new SoRotation;
    SoLightModel * lightModel = new SoLightModel;

    SoNode *ivRoot = graspItGUI->getIVmgr()->getViewer()->getSceneGraph();

    soMaterial->diffuseColor.setValue(1,0,0);

    lightDir->rotation.connectFrom(&graspItGUI->getIVmgr()->getViewer()->getCamera()->orientation);
    lightSep->addChild(lightDir);
    lightSep->addChild(graspItGUI->getIVmgr()->getViewer()->getHeadlight());

    lightModel->model=SoLightModel::PHONG;

    bciWorldViewRoot->addChild(graspItGUI->getIVmgr()->getViewer()->getCamera());
    bciWorldViewRoot->addChild(lightSep);
    bciWorldViewRoot->addChild(lightModel);
    bciWorldViewRoot->addChild(ivRoot);

    renderArea->setSceneGraph(bciWorldViewRoot);
    renderArea->setBackgroundColor(SbColor(1,1,1));
    renderArea->scheduleRedraw();
    renderArea->render();
    renderArea->show();
}


BCIWidgetManager::~BCIWidgetManager()
{
}

#endif // BCIWORLDVIEW_H

