#include "BCI/states/bookmarkState.h"

#include "BCI/states/executionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>


BookmarkState::BookmarkState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("BookmarkState", parent), bciControlWindow(_bciControlWindow),
      csm(_csm)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();
}


void BookmarkState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Home State");

    csm->clearTargets();
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_execution.png"), -0.7 , 0.7, 0.0);

    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                      -1.4, -0.6, 0.0, QString("Go\nHome")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -0.8, 0.0, QString("Drop in\nBin")));

    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -1.0, 0.0, QString("Back")));


    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToExecuteGoHomeState()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(emit_goToExecuteGoToBinState()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(emit_goToHomeState()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);

}


void BookmarkState::emit_goToExecuteGoHomeState()
{
    emit goToExecuteGoHomeState();
}


void BookmarkState::emit_goToExecuteGoToBinState()
{
    emit goToExecuteGoToBinState();
}

void BookmarkState::emit_goToHomeState()
{
    emit goToHomeState();
}

void BookmarkState::onExitImpl(QEvent *e)
{   SoDB::writelock();
    csm->control_scene_separator->removeChild(csm->pipeline->sprite_root);
    SoDB::writeunlock();
    csm->next_target=0;
    delete csm->pipeline;
    executionView->hide();


    std::cout << "Finished onExit of Bookmark State." << std::endl;
}



