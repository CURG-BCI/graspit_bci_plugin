#include "BCI/states/stoppedExecutionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <Inventor/nodes/SoAnnotation.h>

StoppedExecutionState::StoppedExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent)
    :State("StoppedExecutionState", parent), bciControlWindow(_bciControlWindow), csm(_csm)
{
    stoppedExecutionView = new StoppedExecutionView(bciControlWindow->currentFrame);
    stoppedExecutionView->hide();
    alexaSub = n->subscribe("AlexaDetectedPhrases", 1000, &StoppedExecutionState::alexaCB, this);
    alexaPub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
}


void StoppedExecutionState::onEntryImpl(QEvent *e)
{

    std_msgs::String str;
    str.data = "Continue,Start Over";
    alexaPub.publish(str);

    stoppedExecutionView->show();
    bciControlWindow->currentState->setText("Execution");

    csm->clearTargets();
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_paused_execution.png"), pipeline_x, 0.7, 0.0);

    csm->addNewTarget(QString("target_active.png"), btn_x-0.5*btn_width, btn_y, 0.0, QString("Continue"), this, SLOT(onContinueExecutionClicked()));
    csm->addNewTarget(QString("target_background.png"), btn_x+0.5*btn_width, btn_y, 0.0, QString("Start\nOver"), this, SLOT(onStartOverClicked()));
}

void StoppedExecutionState::alexaCB(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "Continue") {
        onContinueExecutionClicked();
    } else if(msg->data == "Start Over") {
        onStartOverClicked();
    }
}


void StoppedExecutionState::onExitImpl(QEvent *e)
{       SoDB::writelock();
        csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
        SoDB::writeunlock();
    delete csm->pipeline;
    csm->next_target=0;
    stoppedExecutionView->hide();

}
