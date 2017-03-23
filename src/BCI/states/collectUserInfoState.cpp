#include "BCI/states/collectUserInfoState.h"

#include "BCI/utils/BCILogger.h"



CollectUserInfoState::CollectUserInfoState(ros::NodeHandle *n): State("CollectUserInfoState")
{
    alexaPub = n->advertise<std_msgs::String>("AlexaValidPhrases", 5);
}

void CollectUserInfoState::onEntryImpl(QEvent *e)
{

    std_msgs::String str;
    str.data = "";
    alexaPub.publish(str);

    settingsUI = new QFormLayout();

    deviceOptions = new QComboBox();
    deviceOptions->insertItem("SEMG-BehindEar");
    deviceOptions->insertItem("SEMG-Forearm");
    deviceOptions->insertItem("Microphone");
    deviceOptions->insertItem("Other");


    subjectName = new QLineEdit();
    subjectName->setPlaceholderText(QString("Subject name"));

    targetObject = new QFormLayout();
    detergent = new QCheckBox("Laundry Detergent");
    shaving = new QCheckBox("Shaving cream");
    shampoo = new QCheckBox("Shampoo");
    targetObject->addRow(detergent);
    targetObject->addRow(shaving);
    targetObject->addRow(shampoo);
    finalTargetObject = new QGroupBox();
    finalTargetObject->setLayout(targetObject);

    goalAction = new QComboBox();
    goalAction->insertItem("Pick up object(s)");
    goalAction->insertItem("Pick up object(s) and drop in bin");

    commentInput = new QTextEdit();

    finishedButton = new QPushButton("Submit");


    settingsUI->addRow(tr("&Subject name: "), subjectName);
    settingsUI->addRow(tr("&Device: "), deviceOptions);
    settingsUI->addRow(tr("&Target object(s): "), finalTargetObject);
    settingsUI->addRow(tr("&Goal: "), goalAction);
    settingsUI->addRow(tr("&Comments: "), commentInput);
    settingsUI->addRow(tr(""), finishedButton);

    finalSettingsUI = new QGroupBox();
    finalSettingsUI->setLayout(settingsUI);



    QObject::connect(finishedButton, SIGNAL(clicked()), this, SLOT(onFinishedEnteringData()));

    finalSettingsUI->show();

}

void CollectUserInfoState::onFinishedEnteringData()
{
    finalSettingsUI->close();

    QString device = QString(deviceOptions->currentText());

    subjectName->selectAll();
    QString name = QString(subjectName->selectedText());

    QString detergentString = "";
    QString shavingString = "";
    QString shampooString = "";
    if (detergent->isChecked())
    {
        detergentString = "Laundry detergent,";
    }
    if (shaving->isChecked())
    {
        shavingString = "Shaving cream,";
    }
    if (shampoo->isChecked())
    {
        shampooString ="Shampoo";
    }

    QString targetName = detergentString + shavingString + shampooString;

    QString goal = QString(goalAction->currentText());

    commentInput->selectAll();
    QString comments = QString(commentInput->selectedText());

    BCILogger::getInstance()->writeExperimentSettings("Name", name);
    BCILogger::getInstance()->writeExperimentSettings("Device", device);
    BCILogger::getInstance()->writeExperimentSettings("Target Object", targetName);
    BCILogger::getInstance()->writeExperimentSettings("Goal", goal);
    BCILogger::getInstance()->writeExperimentSettings("Experiment comments", comments);

    BCIService::getInstance()->emitFinishedCollectingUserInfo();

}
