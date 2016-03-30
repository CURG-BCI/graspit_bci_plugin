#include "BCI/states/stoppedExecutionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include <Inventor/nodes/SoAnnotation.h>

StoppedExecutionState::StoppedExecutionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    :State("StoppedExecutionState", parent), bciControlWindow(_bciControlWindow), csm(_csm)
{
    stoppedExecutionView = new StoppedExecutionView(bciControlWindow->currentFrame);
    stoppedExecutionView->hide();
}


void StoppedExecutionState::onEntry(QEvent *e)
{
    stoppedExecutionView->show();
    bciControlWindow->currentState->setText("Execution");

    csm->clearTargets();
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_paused_execution.png"), -0.7 , 0.7, 0.0);
    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                       -1.4, -1.0, 0.0, QString("Continue")));
    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                       -1.4, -0.8, 0.0, QString("Start\nOver")));

    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(onContinueExecutionClicked()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(onStartOverClicked()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    state_timer.start();
}


void StoppedExecutionState::onExit(QEvent *e)
{       SoDB::writelock();
        csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
        SoDB::writeunlock();
    delete csm->pipeline;
    csm->next_target=0;
    stoppedExecutionView->hide();

     float time=(float) state_timer.elapsed()/1000;
     std::cout<<"!!!!!!!!!!!Elapsed Time is: "<<time<<std::endl;

     QFile log("/home/srihari/ros/graspit_bci_ws/src/graspit_bci_plugin/log.txt");
     if(log.open(QIODevice::ReadWrite | QIODevice::Text|QIODevice::Append))
     {
         std::cout<<"File Writer!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
         QTextStream stream( &log );
         stream << "Time Elapsed in Stopped Execution State: " <<time<<" Seconds."<< endl;
 }


}
