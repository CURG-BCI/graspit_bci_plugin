#ifndef TRANSLATIONSTATE_H
#define TRANSLATIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/ManualAction.h"

class TranslationState: public State
{
   Q_OBJECT

public:
    TranslationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void positiveX();
    void negativeX();
    void positiveY();
    void negativeY();
    void positiveZ();
    void negativeZ();
    void onGoBack();

signals:
    void goToManualState();
    void goToExecuteTranslationState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    actionlib::SimpleActionClient<graspit_msgs::ManualAction> translationActionClient;


    void executeTranslation(int32_t axis, int32_t direction);

    void executeTranslationCallback(const actionlib::SimpleClientGoalState& state,
                                   const graspit_msgs::ManualResultConstPtr& result);
};


#endif
