#ifndef CONTROLLERSCENEMGR_H
#define CONTROLLERSCENEMGR_H


#include <vector>
#include "qobject.h"
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <Inventor/nodes/SoCallback.h>
#include <Inventor/events/SoMouseButtonEvent.h>
#include <Inventor/nodes/SoEventCallback.h>
class Cursor;
class Target;
class SoAnnotation;
class Pipeline;

enum CursorState{SPINNING = 0, MOVING_SLOW = 1, MOVING_FAST = 2};



class ControllerSceneManager: public QObject {

    Q_OBJECT ;


protected:

    int state;

    //Cursor *cursor;
    std::vector<std::shared_ptr<Target>> targets;
    std::vector<std::shared_ptr<Target>> temp_targets;

    boost::recursive_mutex mtx_;

    void lock() {
        mtx_.lock();
    }
    void unlock() {
        mtx_.unlock();
    }
    bool try_lock() {
        return mtx_.try_lock();
    }



public:
    ControllerSceneManager(SoAnnotation *control_scene_separator_);
    Pipeline* pipeline;
    virtual ~ControllerSceneManager() {};
    void addTarget(std::shared_ptr<Target> t);
    void clearTargets();
    void setCursorPosition(double x, double y,  double theta);
    static void handleMouseButtonEvent(void *,SoEventCallback *eventCB);

    int next_target;
    static ControllerSceneManager *current_control_scene_manager;
    SoAnnotation * control_scene_separator;

//    std::shared_ptr<Target> addNewTarget(QString filename,
//                                         double x,
//                                         double y,
//                                         double theta,
//                                         QString target_text,
//                                         const QObject *receiver,
//                                         const char* slot);
    std::shared_ptr<Target> addNewTarget(QString filename,
                                         double x,
                                         double y,
                                         double theta,
                                         QString target_text,
                                         const QObject *receiver,
                                         const char* slot,
                                         QString inactive_filename="target_background.png",
                                         QString active_filename="target_active.png");

public slots:

    void update();
    void setState(int state);



};



#endif // CONTROLLERSCENEMGR_H
