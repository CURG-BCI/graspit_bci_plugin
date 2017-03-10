#ifndef GRASPSELECTIONSTATE_H
#define GRASPSELECTIONSTATE_H

#include "BCI/state.h"
#include <QSignalTransition>
#include "include/debug.h"
#include "bciControlWindow.h"
#include "state_views/graspSelectionView.h"

class GraspSelectionState: public State
{
    Q_OBJECT

public:
    GraspSelectionState(BCIControlWindow *_bciControlWindow,ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent = 0 );
    QString stateName;
    BCIControlWindow *bciControlWindow;

    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);
    void alexaCB(const std_msgs::String::ConstPtr& msg);

protected:
    GraspSelectionView *graspSelectionView;
    ControllerSceneManager *csm;
    std::shared_ptr<Target>  t1;
    std::shared_ptr<Target>  t2;
    ros::Subscriber alexaSub;

    void _updateCurrentGraspView();
    void _updateNextGraspView();
    void showCurrentGrasp();

public slots:
    void onNext();
    void render();

    void emit_goToGraspPlanningState() {emit goToGraspPlanningState();}
    void emit_goToObjectSelectionState(){emit goToObjectSelectionState();}
    void emit_goToConfirmationState(){emit goToConfirmationState();}


signals:
    void goToObjectSelectionState();
    void goToConfirmationState();
    void goToGraspPlanningState();



};


#endif // GRASPSELECTIONSTATE_H
