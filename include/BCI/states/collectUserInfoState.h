#ifndef COLLECT_USER_INFO_STATE_H
#define COLLECT_USER_INFO_STATE_H


#include <QSignalTransition>
#include "include/debug.h"
#include "bciControlWindow.h"
#include "BCI/state.h"

#include <QComboBox>
#include <QGroupBox>
#include <QFormLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QTextEdit>

#include <QCheckBox>



class CollectUserInfoState: public State
{
    Q_OBJECT

public:
    CollectUserInfoState(ros::NodeHandle *n);

public slots:
    virtual void onEntryImpl(QEvent *e);

    void onFinishedEnteringData();


protected:
    QGroupBox *finalSettingsUI;
    QFormLayout *settingsUI;
    QComboBox *deviceOptions;
    QLineEdit *subjectName;
    QFormLayout *targetObject;
    QGroupBox *finalTargetObject;
    QCheckBox *detergent;
    QCheckBox *shaving;
    QCheckBox *shampoo;

    QComboBox *goalAction;

    QTextEdit *commentInput;
    QPushButton * finishedButton;
private:
    ros::Publisher alexaPub;
};


#endif


