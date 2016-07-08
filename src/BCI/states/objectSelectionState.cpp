#include "BCI/states/objectSelectionState.h"
#include "BCI/bciService.h"
#include "state_views/objectSelectionView.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"

#include "include/graspitCore.h"
#include <QGLWidget>


using bci_experiment::world_element_tools::getWorld;
using bci_experiment::GraspManager;
using bci_experiment::WorldController;


ObjectSelectionState::ObjectSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm,
                                           QState* parent):
    State("ObjectSelectionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm)
{
    objectSelectionView = new ObjectSelectionView(this,bciControlWindow->currentFrame);
    objectSelectionView->hide();
}


void ObjectSelectionState::onEntryImpl(QEvent *e)
{
    objectSelectionView->show();

    WorldController::getInstance()->highlightAllBodies();
    GraspableBody *currentTarget = GraspManager::getInstance()->getCurrentTarget();
    WorldController::getInstance()->highlightCurrentBody(currentTarget);
    GraspManager::getInstance()->showRobots(false);
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_object_selection.png"), -0.7 , 0.7, 0.0);
    csm->clearTargets();
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                      -1.4, -0.6, 0.0, QString("Next\nObject")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -0.8, 0.0, QString("Select\nObject")));


//    std::shared_ptr<Target>  t4 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
//                                                                       QString("target_background.png"),
//                                                                      -1.4, -1 , 0.0, QString("Back")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(onNext()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onSelect()));
//    QObject::connect(t4.get(), SIGNAL(hit()), this, SLOT(onGoHome()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addNewTarget(QString("target_background.png"), -1.4, -1.0 , 0.0, QString("Back"), this, SLOT(onGoHome()));
}


void ObjectSelectionState::onExitImpl(QEvent *e)
{

    SoDB::writelock();
     csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
     SoDB::writeunlock();
     csm->next_target=0;
    delete csm->pipeline;

    objectSelectionView->hide();


    csm->clearTargets();

    GraspManager::getInstance()->clearGrasps();
    GraspManager::getInstance()->getGraspsFromDB();


}

void ObjectSelectionState::onNext()
{
    static QTime activeTimer;
    qint64 minElapsedMSecs = 1200;
    if(!activeTimer.isValid() || activeTimer.elapsed() >= minElapsedMSecs)
    {

        activeTimer.start();
        GraspableBody *newTarget = GraspManager::getInstance()->incrementCurrentTarget();
        WorldController::getInstance()->highlightCurrentBody(newTarget);
    }
    csm->setCursorPosition(-1,0,0);
}

void ObjectSelectionState::onSelect()
{
    BCIService::getInstance()->emitGoToNextState1();
}

void ObjectSelectionState::onGoHome()
{
    emit goToHomeState();
}
