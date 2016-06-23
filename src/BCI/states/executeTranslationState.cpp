#include "BCI/states/executeTranslationState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "BCI/graspManager.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>
#include "graspit_msgs/TranslationGoal.h"

ExecuteTranslationState::ExecuteTranslationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("TranslationState", parent),
      bciControlWindow(_bciControlWindow),
      csm(_csm),
      translationActionClient("translation_action", true)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();


}

void ExecuteTranslationState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Translation State");

    csm->clearTargets();

    csm->addNewTarget(QString("target_active.png"), -1.4, -1.0 , 0.0, QString("STOP!"), this, SLOT(onStop()));


}

void ExecuteTranslationState::onStop()
{
    translationActionClient.cancelAllGoals();
    emit goToTranslationState();
}


void ExecuteTranslationState::onExitImpl(QEvent *e)
{
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Translation State." << std::endl;
}
