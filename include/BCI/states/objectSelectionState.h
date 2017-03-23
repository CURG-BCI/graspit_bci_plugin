#ifndef OBJECTSELECTIONSTATE_H
#define OBJECTSELECTIONSTATE_H

#include <QObject>
#include <QSignalTransition>
#include "BCI/state.h"
#include "BCI/graspManager.h"
#include "bciControlWindow.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

class ObjectSelectionView;

class ObjectSelectionState:public State
{
    Q_OBJECT

public:
    ObjectSelectionState(BCIControlWindow *_bciControlWindow, ControllerSceneManager *_csm, ros::NodeHandle *n, QState* parent = 0 );
    virtual void onEntryImpl(QEvent *e);
    virtual void onExitImpl(QEvent *e);
    void alexaCB(const std_msgs::String::ConstPtr& msg);

private:
    BCIControlWindow *bciControlWindow;
    ObjectSelectionView *objectSelectionView;
    BCIService *bciService;
    ControllerSceneManager *csm;
    ros::Subscriber alexaSub;
    ros::Publisher alexaPub;

public slots:

    void onNext();
    void onSelect();
    void onGoHome();


signals:
    void goToNextState();
    void goToHomeState();

};

#endif // OBJECTSELECTIONSTATE_H
