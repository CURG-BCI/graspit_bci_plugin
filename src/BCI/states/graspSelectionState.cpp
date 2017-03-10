#include "BCI/states/graspSelectionState.h"
#include "BCI/bciService.h"
#include "BCI/graspManager.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include<Inventor/nodes/SoAnnotation.h>
#include "BCI/controller_scene/sprites.h"
#include <QPushButton>

using bci_experiment::GraspManager;

GraspSelectionState::GraspSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent):
    State("GraspSelectionState", parent),
    bciControlWindow(_bciControlWindow),
    csm(_csm),
    stateName(QString("Grasp Selection"))
{
    addSelfTransition(BCIService::getInstance(),SIGNAL(plannerUpdated()), this, SLOT(render()));
    addSelfTransition(GraspManager::getInstance(),SIGNAL(render()), this, SLOT(render()));

    graspSelectionView = new GraspSelectionView(bciControlWindow->currentFrame);
    graspSelectionView->hide();

    alexaSub = n->subscribe("AlexaDetectedPhrases", 1000, &GraspSelectionState::alexaCB, this);

    ros::Publisher pub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
    std_msgs::String str;
    str.data = "Next Grasp,Select Grasp,Plan New Grasps,Back";
    pub.publish(str);
}

void GraspSelectionState::onEntryImpl(QEvent *e)
{

    WorldController::getInstance()->unhighlightAllBodies();
    GraspManager::getInstance()->startGraspReachabilityAnalysis();
    GraspManager::getInstance()->showRobots(true);

    graspSelectionView->show();
    bciControlWindow->currentState->setText(stateName);
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_selection.png"), pipeline_x, 0.7, 0.0);
    render();
    csm->clearTargets();

    //Uncomment to get selecting grasp back
    csm->addNewTarget(QString("target_active.png"), btn_x-1.5*btn_width, btn_y, 0.0, QString("Next\nGrasp"), this, SLOT(onNext()));
    csm->addNewTarget(QString("target_background.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Select\nGrasp"), this, SLOT(emit_goToConfirmationState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Plan New\nGrasps"), this, SLOT(emit_goToGraspPlanningState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+1.5*btn_width, btn_y, 0.0, QString("Back"), this, SLOT(emit_goToObjectSelectionState()));

    //do this to ensure that the side views are reset.
    GraspManager::getInstance()->decrementGraspIndex();
    onNext();
}

void GraspSelectionState::alexaCB(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "Next Grasp") {
        onNext();
    } else if(msg->data == "Select Grasp") {
        emit_goToConfirmationState();
    } else if(msg->data == "Plan New Grasps") {
        emit_goToGraspPlanningState();
    } else if(msg->data == "Back") {
        emit_goToObjectSelectionState();
    }
}

void GraspSelectionState::onExitImpl(QEvent *e)
{
    GraspManager::getInstance()->startGraspReachabilityAnalysis();
    csm->clearTargets();
    SoDB::writelock();
    csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
    SoDB::writeunlock();
    delete csm->pipeline;
    csm->next_target=0;
    graspSelectionView->hide();

}

void GraspSelectionState::_updateCurrentGraspView()
{
    const GraspPlanningState * currentGrasp = GraspManager::getInstance()->getCurrentGrasp();

    if(currentGrasp)
    {
        Hand *graspDemoHand = GraspManager::getInstance()->getHand();
        graspSelectionView->showSelectedGrasp(graspDemoHand, currentGrasp);
        QString graspID;
        bciControlWindow->currentState->setText(stateName +"- Grasp: " + graspID.setNum(currentGrasp->getAttribute("graspId")) );
    }
    else
    {
       std::cout<<"GraspSelectionState::_updateCurrentGraspView() in else"<<std::endl;
       Hand *graspDemoHand = GraspManager::getInstance()->getHand();
       graspSelectionView->showSelectedGrasp(graspDemoHand, NULL);

    }
}

void GraspSelectionState::_updateNextGraspView()
{
    const GraspPlanningState *nextGrasp = GraspManager::getInstance()->getNextGrasp();
    if(nextGrasp)
    {
        Hand *graspDemoHand = GraspManager::getInstance()->getHand();
        graspSelectionView->showNextGrasp(graspDemoHand, nextGrasp);
    }
    else
    {
       Hand *graspDemoHand = GraspManager::getInstance()->getHand();
       graspSelectionView->showNextGrasp(graspDemoHand, NULL);
    }
}

void GraspSelectionState::showCurrentGrasp()
{
    const GraspPlanningState * currentGrasp = GraspManager::getInstance()->getCurrentGrasp();
    if (currentGrasp)
    {
        currentGrasp->execute(GraspManager::getInstance()->getHand());
    }
}

void GraspSelectionState::onNext()
{
    GraspManager::getInstance()->incrementGraspIndex();

    _updateCurrentGraspView();
    _updateNextGraspView();
    showCurrentGrasp();

    csm->setCursorPosition(-1,0,0);
}

void GraspSelectionState::render()
{
    ROS_INFO("GraspSelectionState::onPlannerUpdated()");
    static QTime activeTimer;
    qint64 minElapsedMSecs = 300;
    if(!activeTimer.isValid() || activeTimer.elapsed() >= minElapsedMSecs)
    {
        GraspManager::getInstance()->resetGraspIndex();

        _updateCurrentGraspView();
        _updateNextGraspView();
        showCurrentGrasp();
    }
    if(!GraspManager::getInstance()->getCurrentGrasp())
    {
        QObject::disconnect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToConfirmationState()));
        QObject::disconnect(t2.get(), SIGNAL(hit()), this, SLOT(onNext()));
    }

    GraspManager::getInstance()->renderPending = false;
}



