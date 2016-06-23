#include "BCI/states/homeState.h"

#include "BCI/states/executionState.h"
#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "include/EGPlanner/searchState.h"
#include <Inventor/nodes/SoAnnotation.h>


HomeState::HomeState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent)
    : State("HomeState", parent), bciControlWindow(_bciControlWindow),
      csm(_csm)
{
    executionView = new ExecutionView(bciControlWindow->currentFrame);
    executionView->hide();
}


void HomeState::onEntryImpl(QEvent *e)
{
    executionView->show();
    bciControlWindow->currentState->setText("Home State");

    csm->clearTargets();

    std::shared_ptr<Target>  t1 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_active.png"),
                                                                      -1.4, -0.6, 0.0, QString("Grasp\nPipeline")));

    std::shared_ptr<Target>  t2 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -0.8, 0.0, QString("Manual\nMovement")));

    std::shared_ptr<Target>  t3 = std::shared_ptr<Target> (new Target(csm->control_scene_separator,
                                                                       QString("target_background.png"),
                                                                      -1.4, -1.0, 0.0, QString("Bookmarks")));


    QObject::connect(t1.get(), SIGNAL(hit()), this, SLOT(emit_goToObjectRecognitionState()));
    QObject::connect(t2.get(), SIGNAL(hit()), this, SLOT(emit_goToManualState()));
    QObject::connect(t3.get(), SIGNAL(hit()), this, SLOT(emit_goToBookmarkState()));

    csm->addTarget(t1);
    csm->addTarget(t2);
    csm->addTarget(t3);

}

void HomeState::emit_goToObjectRecognitionState()
{
    emit goToObjectRecognitionState();
}

void HomeState::emit_goToManualState()
{
    emit goToManualState();
}


void HomeState::emit_goToBookmarkState()
{
    emit goToBookmarkState();
}

void HomeState::onExitImpl(QEvent *e)
{
    std::cout << "Starting onExit of Home State." << std::endl;
    csm->next_target=0;
    executionView->hide();


    std::cout << "Finished onExit of Home State." << std::endl;
}



