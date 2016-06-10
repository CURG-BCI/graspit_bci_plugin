#include "BCI/BCIStateMachine.h"

#include "BCI/states/objectSelectionState.h"
#include "BCI/states/graspSelectionState.h"
#include "BCI/states/placementLocationSelectionState.h"
#include "BCI/states/confirmationState.h"
#include "BCI/states/executionState.h"
#include "BCI/states/stoppedExecutionState.h"
#include "BCI/states/objectRecognitionState.h"
#include "BCI/states/collectUserInfoState.h"
#include "BCI/states/graspPlanningState.h"

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

    collectUserInfoState->addStateTransition(bciService, SIGNAL(finishedCollectingUserInfo()), objectRecognitionState);
    objectRecognitionState->addStateTransition(bciService, SIGNAL(finishedRecognition()), objectSelectionState);

    objectSelectionState->addStateTransition(bciService,SIGNAL(goToNextState1()), graspSelectionState);
    objectSelectionState->addStateTransition(objectSelectionState,SIGNAL(goToNextState()), graspSelectionState);
    objectSelectionState->addSelfTransition(bciService, SIGNAL(exec()), objectSelectionState, SLOT(onSelect()));
    objectSelectionState->addSelfTransition(bciService, SIGNAL(rotLat()), objectSelectionState, SLOT(onNext()));

    objectSelectionState->addStateTransition(bciService,SIGNAL(goToPreviousState()), objectRecognitionState);

    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToObjectSelectionState()), objectSelectionState);
    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToConfirmationState()), confirmationState);
    graspSelectionState->addStateTransition(graspSelectionState, SIGNAL(goToGraspPlanningState()), planGraspState);

    planGraspState->addStateTransition(planGraspState, SIGNAL(goToGraspSelectionState()), graspSelectionState);

    confirmationState->addStateTransition(confirmationState, SIGNAL(goToExecutionState()), executionState);
    confirmationState->addStateTransition(confirmationState, SIGNAL(goToPreviousState()), graspSelectionState);

    executionState->addStateTransition(executionState, SIGNAL(goToStoppedExecutionState()), stoppedExecutionState);

    stoppedExecutionState->addStateTransition(stoppedExecutionState, SIGNAL(goToExecutionState()), executionState);
    stoppedExecutionState->addStateTransition(stoppedExecutionState, SIGNAL(goToObjectSelectionState()), objectSelectionState);

    stateMachine.addState(collectUserInfoState);
    stateMachine.addState(objectRecognitionState);
    stateMachine.addState(objectSelectionState);
    stateMachine.addState(graspSelectionState);
    stateMachine.addState(finalGraspSelectionState);
    stateMachine.addState(confirmationState);
    stateMachine.addState(executionState);
    stateMachine.addState(stoppedExecutionState);
    stateMachine.addState(planGraspState);

    stateMachine.setInitialState(collectUserInfoState);
}

void BCIStateMachine::start()
{
    stateMachine.start();
    bciControlWindow->currentFrame->show();
}


