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
    csm->pipeline=new Pipeline(csm->control_scene_separator, QString("pipeline_grasp_execution.png"), -1.2 , 0.7, 0.0);

    csm->addNewTarget(QString("target_active.png"), btn_x-btn_width, btn_y, 0.0, QString("Go\nHome"), this, SLOT(emit_goToExecuteGoHomeState()));
    csm->addNewTarget(QString("target_background.png"), btn_x, btn_y, 0.0, QString("Drop in\nBin"), this, SLOT(emit_goToExecuteGoToBinState()));
    csm->addNewTarget(QString("target_background.png"), btn_x+btn_width, btn_y, 0.0, QString("Back"), this, SLOT(emit_goToHomeState()));
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
