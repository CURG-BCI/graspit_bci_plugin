#ifndef BCI_LOGGER_H
#define BCI_LOGGER_H

#include <QFile>

class QString;

class BCILogger
{

public:
    ~BCILogger();
    static BCILogger* getInstance();

    void appendToTimingLog(QString msg);
    void writeExperimentSettings(QString key, QString value);

private:
        //singleton pattern, single static instance of the class
        static BCILogger * bciLoggerInstance;

        //Where the logs are going to be placed.
        QFile *timingLog;

        //log subject name, device used, etc
        QFile *experimentSettingsLog;

        //this is singleton, so constructor must be private.
        BCILogger();
};

#endif // BCISERVICE_H
