#include "BCI/BCIStateMachine.h"

#include "BCI/states/homeState.h"
#include "BCI/states/objectSelectionState.h"
#include "BCI/states/graspSelectionState.h"
#include "BCI/states/placementLocationSelectionState.h"
#include "BCI/states/confirmationState.h"
#include "BCI/states/executionState.h"
#include "BCI/states/stoppedExecutionState.h"
#include "BCI/states/objectRecognitionState.h"
#include "BCI/states/collectUserInfoState.h"
#include "BCI/states/graspPlanningState.h"
#include "BCI/states/homeState.h"
#include "BCI/states/stoppedGoHomeState.h"
#include "BCI/states/stoppedGoToBinState.h"
#include "BCI/states/executeGoHomeState.h"
#include "BCI/states/executeGoToBinState.h"
#include "BCI/states/translationState.h"
#include "BCI/states/executeTranslationState.h"
#include "BCI/states/bookmarkState.h"
#include "BCI/states/manualState.h"

#include "BCI/bciService.h"

BCIStateMachine::BCIStateMachine(BCIControlWindow *_bciControlWindow, BCIService *_bciService):
    bciControlWindow(_bciControlWindow),
    bciService(_bciService)
{

    csm = bciService->csm;
    ros::NodeHandle *n = new ros::NodeHandle("");
    CollectUserInfoState *collectUserInfoState = new CollectUserInfoState();
    ObjectRecognitionState *objectRecognitionState = new ObjectRecognitionState(bciControlWindow, csm, n);
    ObjectSelectionState *objectSelectionState = new ObjectSelectionState(bciControlWindow, csm);
    GraspSelectionState *graspSelectionState = new GraspSelectionState(bciControlWindow, csm);
    GraspSelectionState *finalGraspSelectionState = new GraspSelectionState(bciControlWindow, csm);
    ConfirmationState *confirmationState = new ConfirmationState(bciControlWindow, csm);
    ExecutionState *executionState = new ExecutionState(bciControlWindow, csm, n);
    StoppedExecutionState *stoppedExecutionState = new StoppedExecutionState(bciControlWindow, csm);
    PlanGraspState *planGraspState = new PlanGraspState(bciControlWindow, csm);
    HomeState *homeState = new HomeState(bciControlWindow, csm);
    StoppedGoHomeState *stoppedGoHomeState = new StoppedGoHomeState(bciControlWindow, csm);
    StoppedGoToBinState *stoppedGoToBinState = new StoppedGoToBinState(bciControlWindow, csm);
    ExecuteGoHomeState *executeGoHomeState = new ExecuteGoHomeState(bciControlWindow, csm, n);
    ExecuteGoToBinState *executeGoToBinState = new ExecuteGoToBinState(bciControlWindow, csm, n);
    TranslationState *translationState = new TranslationState(bciControlWindow, csm);
    BookmarkState *bookmarkState = new BookmarkState(bciControlWindow, csm);
    ManualState *manualState = new ManualState(bciControlWindow, csm);
    ExecuteTranslationState *executeTranslationState = new ExecuteTranslationState(bciControlWindow, csm);

    collectUserInfoState->addStateTransition(bciService, SIGNAL(finishedCollectingUserInfo()), homeState);
    objectRecognitionState->addStateTransition(bciService, SIGNAL(finishedRecognition()), objectSelectionState);

    homeState->addStateTransition(homeState, SIGNAL(goToObjectRecognitionState()), objectRecognitionState);
    homeState->addStateTransition(homeState, SIGNAL(goToManualState()), manualState);
    homeState->addStateTransition(homeState, SIGNAL(goToBookmarkState()), bookmarkState);

    objectSelectionState->addStateTransition(bciService,SIGNAL(goToNextState1()), graspSelectionState);
    objectSelectionState->addStateTransition(objectSelectionState,SIGNAL(goToNextState()), graspSelectionState);
    objectSelectionState->addSelfTransition(bciService, SIGNAL(exec()), objectSelectionState, SLOT(onSelect()));
    objectSelectionState->addSelfTransition(bciService, SIGNAL(rotLat()), objectSelectionState, SLOT(onNext()));

    objectSelectionState->addStateTransition(bciService,SIGNAL(goToPreviousState()), objectRecognitionState);
    objectSelectionState->addStateTransition(objectSelectionState, SIGNAL(goToHomeState()), homeState);

    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToObjectSelectionState()), objectSelectionState);
    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToConfirmationState()), confirmationState);
    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToGraspPlanningState()), planGraspState);

    planGraspState->addStateTransition(planGraspState, SIGNAL(goToGraspSelectionState()), graspSelectionState);

    confirmationState->addStateTransition(confirmationState, SIGNAL(goToExecutionState()), executionState);
    confirmationState->addStateTransition(confirmationState, SIGNAL(goToPreviousState()), graspSelectionState);

    executionState->addStateTransition(executionState, SIGNAL(goToStoppedExecutionState()), stoppedExecutionState);
    executionState->addStateTransition(executionState, SIGNAL(goToHomeState()), homeState);

    bookmarkState->addStateTransition(bookmarkState, SIGNAL(goToExecuteGoHomeState()), executeGoHomeState);
    bookmarkState->addStateTransition(bookmarkState, SIGNAL(goToExecuteGoToBinState()), executeGoToBinState);
    bookmarkState->addStateTransition(bookmarkState, SIGNAL(goToHomeState()), homeState);

    stoppedExecutionState->addStateTransition(stoppedExecutionState, SIGNAL(goToExecutionState()), executionState);
    stoppedExecutionState->addStateTransition(stoppedExecutionState, SIGNAL(goToObjectSelectionState()), objectSelectionState);

    executeGoHomeState->addStateTransition(executeGoHomeState, SIGNAL(goToStoppedGoHomeState()), stoppedGoHomeState);
    executeGoHomeState->addStateTransition(executeGoHomeState, SIGNAL(goToHomeState()), homeState);

    executeGoToBinState->addStateTransition(executeGoToBinState, SIGNAL(goToStoppedGoToBinState()), stoppedGoToBinState);
    executeGoToBinState->addStateTransition(executeGoToBinState, SIGNAL(goToHomeState()), homeState);

    stoppedGoHomeState->addStateTransition(stoppedGoHomeState, SIGNAL(goToExecuteGoHomeState()), executeGoHomeState);
    stoppedGoHomeState->addStateTransition(stoppedGoHomeState, SIGNAL(goToBookmarkState()), bookmarkState);

    stoppedGoToBinState->addStateTransition(stoppedGoToBinState, SIGNAL(goToExecuteGoToBinState()), executeGoToBinState);
    stoppedGoToBinState->addStateTransition(stoppedGoToBinState, SIGNAL(goToBookmarkState()), bookmarkState);

    translationState->addStateTransition(translationState, SIGNAL(goToManualState()), manualState);
    translationState->addStateTransition(translationState, SIGNAL(goToExecuteTranslationState()), executeTranslationState);

    manualState->addStateTransition(manualState, SIGNAL(goToTranslationState()), translationState);
    manualState->addStateTransition(manualState, SIGNAL(goToHomeState()), homeState);

    executeTranslationState->addStateTransition(executeTranslationState, SIGNAL(goToTranslationState()), translationState);



    stateMachine.addState(collectUserInfoState);
    stateMachine.addState(objectRecognitionState);
    stateMachine.addState(objectSelectionState);
    stateMachine.addState(graspSelectionState);
    stateMachine.addState(finalGraspSelectionState);
    stateMachine.addState(confirmationState);
    stateMachine.addState(executionState);
    stateMachine.addState(stoppedExecutionState);
    stateMachine.addState(planGraspState);
    stateMachine.addState(stoppedGoHomeState);
    stateMachine.addState(stoppedGoToBinState);
    stateMachine.addState(executeGoHomeState);
    stateMachine.addState(executeGoToBinState);
    stateMachine.addState(translationState);
    stateMachine.addState(manualState);
    stateMachine.addState(bookmarkState);
    stateMachine.addState(homeState);
    stateMachine.addState(executeTranslationState);

    //stateMachine.setInitialState(collectUserInfoState);
    stateMachine.setInitialState(homeState);
}

void BCIStateMachine::start()
{
    stateMachine.start();
    bciControlWindow->currentFrame->show();
}


