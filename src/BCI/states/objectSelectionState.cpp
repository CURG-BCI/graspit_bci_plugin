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


ObjectSelectionState::ObjectSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n,
                                           QState* parent):
    State("ObjectSelectionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm)
{
    objectSelectionView = new ObjectSelectionView(this,bciControlWindow->currentFrame);
    objectSelectionView->hide();

    alexaSub = n->subscribe("AlexaDetectedPhrases", 1000, &ObjectSelectionState::alexaCB, this);

    ros::Publisher pub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
    std_msgs::String str;
    str.data = "Next Object,Select Object,Rerun Vision";
    pub.publish(str);
}


void ObjectSelectionState::onEntryImpl(QEvent *e)
{
    objectSelectionView->show();

    WorldController::getInstance()->highlightAllBodies();
    GraspableBody *currentTarget = GraspManager::getInstance()->getCurrentTarget();
    WorldController::getInstance()->highlightCurrentBody(currentTarget);
    GraspManager::getInstance()->showRobots(false);
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_object_selection.png"), pipeline_x, 0.7, 0.0);
    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), btn_x-btn_width, btn_y, 0.0, QString("Next\nObject"), this, SLOT(onNext()));
    csm->addNewTarget(QString("target_background.png"), btn_x, btn_y, 0.0, QString("Select\nObject"), this, SLOT(onSelect()));
    csm->addNewTarget(QString("target_background.png"), btn_x+btn_width, btn_y, 0.0, QString("Rerun\nVision"), this, SLOT(onGoHome()));
}

void ObjectSelectionState::alexaCB(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "Next Object") {
        onNext();
    } else if(msg->data == "Select Object") {
        onSelect();
    } else if(msg->data == "Rerun Vision") {
        onGoHome();
    }
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
    //GraspManager::getInstance()->getGraspsFromDB();
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
