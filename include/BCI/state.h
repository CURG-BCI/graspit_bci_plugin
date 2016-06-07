#ifndef STATE_H
#define STATE_H

#include <QState>
#include <QSignalTransition>
#include <vector>
#include <QTime>
#include <QFile>
class BCIEvent;
class QImage;
class QColor;

class State : public QState
{
    Q_OBJECT
public:
    explicit State( const QString& name, QState* parent = 0 );
    explicit State( const QString& name, const QString& prefix, QState* parent = 0 );

    QString name() const { return m_name; }
    QString prefix() const { return m_prefix; }


    QSignalTransition * addSelfTransition(QObject * sender, const char * signal, const QObject *receiver, const char* slot);
    //Reimplement addTransition interface more safely -- explicitly disallow duplicates.
    QSignalTransition * addStateTransition(QObject * sender, const char * signal,  QAbstractState * target);
    void addStateTransition ( QAbstractTransition * transition );
    QAbstractTransition *addStateTransition ( QAbstractState * target );

public slots:
    void setName( const QString& name ) { m_name = name; }
    void setPrefix( const QString& prefix ) { m_prefix = prefix; }


protected:
    //these should NOT be overwritten.  We want these called for every state for logging etc.
    virtual void onEntry( QEvent* e );
    virtual void onExit( QEvent* e );

    //override these for subclass specific entry/exit actions to occur
    virtual void onEntryImpl( QEvent* e );
    virtual void onExitImpl( QEvent* e );

    QAbstractTransition *checkForDuplicateTransitions(QAbstractTransition * transition);

protected:
    QString m_name;
    QString m_prefix;
    QTime state_timer;
    QFile log_file;

};



#endif

