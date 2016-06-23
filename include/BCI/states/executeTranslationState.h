#ifndef EXECUTETRANSLATIONSTATE_H
#define EXECUTETRANSLATIONSTATE_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/TranslationAction.h"

class ExecuteTranslationState: public State
{
   Q_OBJECT

public:
    ExecuteTranslationState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void onStop();

signals:
    void goToTranslationState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    actionlib::SimpleActionClient<graspit_msgs::TranslationAction> translationActionClient;



};


#endif
