#include "BCI/utils/BCILogger.h"

#include <QTextStream>
#include <QString>
#include <QTime>
#include <QDate>
#include <iostream>
#include <QDir>

#include <QComboBox>
#include <QDialogButtonBox>
#include <QDialog>

BCILogger * BCILogger::bciLoggerInstance = NULL;

BCILogger* BCILogger::getInstance()
{
    if(!bciLoggerInstance)
    {
        bciLoggerInstance = new BCILogger();
    }

    return bciLoggerInstance;
}


void BCILogger::appendToTimingLog(QString msg)
{
    if(!msg.endsWith("\n"))
    {
        msg = msg + QString("\n");
    }

    if(timingLog->open(QIODevice::ReadWrite | QIODevice::Text|QIODevice::Append))
    {
        QTextStream stream( timingLog );
        stream << msg.toStdString().c_str();
    }

    timingLog->close();
}

void BCILogger::writeExperimentSettings(QString key, QString value)
{
    if(experimentSettingsLog->open(QIODevice::ReadWrite | QIODevice::Text|QIODevice::Append))
    {
        QTextStream stream( experimentSettingsLog );
        stream << key.toStdString().c_str();
        stream << ":";
        stream << value.toStdString().c_str();
        stream << "\n";
    }

    experimentSettingsLog->close();
}


BCILogger::BCILogger()
{
    QString currentDate = QString(QDate::currentDate().toString("MM_dd_yyyy"));
    QString currentTime = QString(QTime::currentTime().toString("hh_mm_ss"));
    QString logDir = QString(getenv("LOG_DIR")) +  currentDate + QString("_") + currentTime;

    if(!QDir(logDir).exists())
    {
        QDir().mkdir(logDir);
    }

    QString timingLogFilename = logDir + QString("/timing.csv");
    QString settingsLogFilename =logDir + QString("/settings.yaml");

    timingLog = new QFile(timingLogFilename);
    experimentSettingsLog = new QFile(settingsLogFilename);

    //Initialize header column in logfile.
    appendToTimingLog("Current State, Time(Seconds)\n");
}

BCILogger::~BCILogger()
{
    delete timingLog;
    delete experimentSettingsLog;
 }
