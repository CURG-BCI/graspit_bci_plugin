#ifndef EXECUTEGOTOBIN_H
#define EXECUTEGOTOBIN_H

#include "BCI/state.h"
#include "bciControlWindow.h"
#include "state_views/executionView.h"
#include "graspit_msgs/AfterGraspingAction.h"


class ExecuteGoToBinState: public State
{
   Q_OBJECT

public:
    ExecuteGoToBinState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent=0);

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);

public slots:
    void emit_goToStoppedGoToBinState();

signals:
    void goToStoppedGoToBinState();
    void goToHomeState();

private:
    BCIControlWindow *bciControlWindow;
    ExecutionView *executionView;
    ControllerSceneManager *csm;
    ros::Publisher grasp_stop_execution_publisher;
    ros::Publisher grasp_stop_fingers_execution_publisher;
    ros::ServiceClient action_after_grasping;
    actionlib::SimpleActionClient<graspit_msgs::AfterGraspingAction> afterGraspingActionClient;

    void goToBin();

    void goToBinCallback(const actionlib::SimpleClientGoalState& state,
                                   const graspit_msgs::AfterGraspingResultConstPtr& result);

};


#endif
