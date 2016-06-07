#include "BCI/states/collectUserInfoState.h"

#include "BCI/utils/BCILogger.h"



CollectUserInfoState::CollectUserInfoState(): State("CollectUserInfoState")
{

}

void CollectUserInfoState::onEntryImpl(QEvent *e)
{

    settingsUI = new QFormLayout();

    deviceOptions = new QComboBox();
    deviceOptions->insertItem("SEMG-BehindEar");
    deviceOptions->insertItem("SEMG-Forearm");
    deviceOptions->insertItem("Microphone");
    deviceOptions->insertItem("Other");


    subjectName = new QLineEdit();
    subjectName->setPlaceholderText(QString("Subject name"));

    targetObject = new QComboBox();
    targetObject->insertItem("Laundry detergent");
    targetObject->insertItem("Shaving cream");
    targetObject->insertItem("Shampoo");

    commentInput = new QTextEdit();

    finishedButton = new QPushButton("Submit");


    settingsUI->addRow(tr("&Subject name: "), subjectName);
    settingsUI->addRow(tr("&Device: "), deviceOptions);
    settingsUI->addRow(tr("&Target object: "), targetObject);
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

    QString targetName = QString(targetObject->currentText());

    commentInput->selectAll();
    QString comments = QString(commentInput->selectedText());

    BCILogger::getInstance()->writeExperimentSettings("Name", name);
    BCILogger::getInstance()->writeExperimentSettings("Device", device);
    BCILogger::getInstance()->writeExperimentSettings("Target Object", targetName);
    BCILogger::getInstance()->writeExperimentSettings("Comments", comments);

    //        QTextStream stream( experimentSettingsLog );
    //        stream << "SubjectName: " << QString("Jake") << std::endl;
    //        stream << "Device: " << QString("Microphone") << std::endl;
    //        stream << "Target Object: " << QString("All Bottle") << std::endl;
    //        stream << "Comments: " << QString("n/a") << std::endl;

    BCIService::getInstance()->emitFinishedCollectingUserInfo();

}
