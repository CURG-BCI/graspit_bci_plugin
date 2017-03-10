#include "BCI/state.h"
#include <QDebug>
#include <QImage>
#include <QPainter>
#include <QColor>
#include "BCI/bciService.h"
#include "BCI/utils/BCILogger.h"
#include "include/debug.h"
#include <QDebug>

State::State( const QString& name, QState* parent )
    : QState( parent ),
      m_name( name ),
      m_prefix()
{
    //this->addSelfTransition(this, SIGNAL(exited()), this, SLOT(disconnectOptionChoice()));
}


State::State( const QString& name, const QString& prefix, QState* parent )
    : QState( parent ),
      m_name( name ),
      m_prefix( prefix )
{
    //this->addSelfTransition(this, SIGNAL(exited()), this, SLOT(disconnectOptionChoice()));
}


void State::onEntry( QEvent* e )
{
    Q_UNUSED( e );

    //start the timer here
    state_timer.start();

    // Print out the state we are entering and it's parents
    QString state = m_name;
    State* parent = dynamic_cast<State*>( parentState() );
    while ( parent != 0 )
    {
        state = parent->name() + "->" + state;
        parent = dynamic_cast<State*>( parent->parentState() );
    }
     std::cout << m_prefix.toStdString().c_str() << "Entering state:" << state.toStdString().c_str() << std::endl;

     onEntryImpl(e);
}

void State::onExit( QEvent* e )
{
    Q_UNUSED( e );

     //log timer and state name to logger
    float time=(float) state_timer.elapsed()/1000;
    std::cout<<"onExit --- Elapsed Time is: "<<time<<std::endl;
    QString state = m_name;
    BCILogger::getInstance()->appendToTimingLog(m_name + QString(", ") + QString::number(time));



    // Print out the state we are exiting and it's parents
    State* parent = dynamic_cast<State*>( parentState() );
    while ( parent != 0 )
    {
        state = parent->name() + "->" + state;
        parent = dynamic_cast<State*>( parent->parentState() );
    }
    std::cout  << m_prefix.toStdString().c_str() << "Exiting state:" << state.toStdString().c_str() << std::endl;

    onExitImpl(e);
}


void State::onEntryImpl( QEvent* e )
{
    Q_UNUSED( e );
     std::cout << m_prefix.toStdString().c_str()  << "Entering base state onEntryImpl"  << std::endl;
}

void State::onExitImpl( QEvent* e )
{
    Q_UNUSED( e );
    std::cout << m_prefix.toStdString().c_str() << "Exiting base state onExitImpl"<< std::endl;
}

QSignalTransition * State::addSelfTransition(QObject *sender, const char * signal, const QObject *receiver, const char* slot  )
{
    QSignalTransition * newTransition =
            new QSignalTransition(sender, signal);
    QSignalTransition *duplicate =
            dynamic_cast<QSignalTransition*>(checkForDuplicateTransitions(newTransition));
    if(!duplicate)
        addStateTransition(newTransition);

    else
    {
        delete(newTransition);
        newTransition = duplicate;
    }

    connect(newTransition, SIGNAL(triggered()),
            receiver, slot, Qt::QueuedConnection);
    return newTransition;
}


QAbstractTransition *
State::checkForDuplicateTransitions(QAbstractTransition * transition)
{
    QSignalTransition * signalTransition = dynamic_cast<QSignalTransition *>(transition);
    if(!signalTransition)
    {
        DBGA("State::checkDuplicateTransitions::Non-signal transition cannot be tested");
        return NULL;
    }

    QList<QAbstractTransition *> transitionList = this->transitions();
    for(QList<QAbstractTransition *>::iterator trans = transitionList.begin(); trans != transitionList.end(); ++trans)
    {
        QSignalTransition * testSignalTransition =
                dynamic_cast<QSignalTransition *>(*trans);
        if(!testSignalTransition)
        {
            DBGA("State::checkDuplicateTransitions::Non-signal transition cannot be tested");
            continue;
        }
        if(testSignalTransition != signalTransition && testSignalTransition->signal() == signalTransition->signal() && testSignalTransition->senderObject() == signalTransition->senderObject())
        {
            DBGA("State::checkForDuplicateTransitions::Duplicate signal found::" << QString(testSignalTransition->signal()).toStdString());
            return testSignalTransition;
        }
    }
    return NULL;
}

QSignalTransition *
State::addStateTransition(QObject * sender, const char * signal,  QAbstractState * target)
{
    QSignalTransition * newTransition =
            new QSignalTransition(sender,signal);
    newTransition->setTargetState(target);
    this->addStateTransition(newTransition);
    return newTransition;
}

void
State::addStateTransition( QAbstractTransition * transition )
{
    Q_ASSERT(!checkForDuplicateTransitions(transition));
    addTransition(transition);
}


void State::alexaCB(const std_msgs::String::ConstPtr& msg) {}


