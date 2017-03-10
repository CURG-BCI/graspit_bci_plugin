
#include "BCI/states/confirmationState.h"
#include  <QSignalTransition>
#include <Inventor/nodes/SoAnnotation.h>
#include "BCI/graspManager.h"
#include "BCI/bciService.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"

#include <memory>

using bci_experiment::world_element_tools::getWorld;
using bci_experiment::GraspManager;

ConfirmationState::ConfirmationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent):
        State("ConfirmationState", parent),bciControlWindow(_bciControlWindow),
        csm(_csm)
{
    confirmationView = new ConfirmationView(bciControlWindow->currentFrame);
    this->addSelfTransition(BCIService::getInstance(),SIGNAL(rotLat()), this, SLOT(onNextGrasp()));
    confirmationView->hide();

    ros::Publisher pub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
    std_msgs::String str;
    str.data = "";
    pub.publish(str);
}


void ConfirmationState::onEntryImpl(QEvent *e)
{
    const GraspPlanningState *grasp = GraspManager::getInstance()->getCurrentGrasp();
    Hand * hand = GraspManager::getInstance()->getHand();
    confirmationView->setCurrentGrasp(hand,grasp);
    confirmationView->show();
    bciControlWindow->currentState->setText("Confirmation");
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_confirmation.png"), pipeline_x, 0.7, 0.0);

//    csm->addNewTarget(QString("target_active.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Confirm\nGrasp"), this, SLOT(emit_goToExecutionState()));
//    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Back"), this, SLOT(emit_goToPreviousState()));

    emit_goToExecutionState();

}

void ConfirmationState::onNextGrasp(QEvent *e)
{

}

void ConfirmationState::onExitImpl(QEvent * e)
{
    csm->clearTargets();
    SoDB::writelock();
     csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
     SoDB::writeunlock();
     csm->next_target=0;
    delete csm->pipeline;
    Q_UNUSED(e);
    confirmationView->hide();

    std::cout << "Finished onExit of Object Selection State." << std::endl;

}
