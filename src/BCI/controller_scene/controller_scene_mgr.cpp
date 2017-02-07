#include "BCI/controller_scene/controller_scene_mgr.h"
#include "BCI/controller_scene/sprites.h"
#include "include/debug.h"
#include "Inventor/nodes/SoAnnotation.h"
#include <Inventor/SoDB.h>
#include "BCI/bciService.h"
#include "Inventor/nodes/SoEventCallback.h"
#include <Inventor/events/SoMouseButtonEvent.h>
#include "include/graspitCore.h"
#include "include/ivmgr.h"
#include <Inventor/nodes/SoPerspectiveCamera.h>

ControllerSceneManager *ControllerSceneManager::current_control_scene_manager = NULL;

ControllerSceneManager::ControllerSceneManager(SoAnnotation *control_scene_separator_)
    : control_scene_separator(control_scene_separator_),
      state(CursorState::SPINNING)
{
    SoEventCallback *mouseEventCB = new SoEventCallback;
    mouseEventCB->addEventCallback(SoMouseButtonEvent::getClassTypeId(), handleMouseButtonEvent);
    control_scene_separator->addChild(mouseEventCB);
    //pipeline=new Pipeline(control_scene_separator, QString("pipeline_grasp_confirmation.png"), -0.7 , 0.7, 0.0);
    //cursor = new Cursor(control_scene_separator, QString("cursor_scaled.png"), .8, -1.0, 0.0);
    current_control_scene_manager = this;
    next_target=0;
}

void ControllerSceneManager::addTarget(std::shared_ptr<Target> t)
{
    this->lock();
    targets.push_back(t);
    this->unlock();
}

std::shared_ptr<Target> ControllerSceneManager::addNewTarget(QString filename,
                                                             double x,
                                                             double y,
                                                             double theta,
                                                             QString target_text,
                                                             const QObject *receiver,
                                                             const char* slot,
                                                             QString inactive_filename,
                                                             QString active_filename)
{
    std::shared_ptr<Target>  target = std::shared_ptr<Target> (new Target(control_scene_separator,
                                                                          filename,
                                                                          x, y , theta, target_text,
                                                                          inactive_filename=inactive_filename,
                                                                          active_filename=active_filename));

    QObject::connect(target.get(), SIGNAL(hit()), receiver, slot);
    this->lock();
    targets.push_back(target);
    this->unlock();
    return target;
}

void ControllerSceneManager::clearTargets()
{
    this->lock();
    for(int i = 0; i < targets.size(); i++)
    {
        targets[i]->valid = false;
    }
    for(int i = 0; i < temp_targets.size(); i++)
    {
        temp_targets[i]->valid = false;
    }
    //targets.clear();
    // this->setCursorPosition(-1,0,0);
    this->unlock();
}

void ControllerSceneManager::setCursorPosition(double x, double y, double theta)
{
    //cursor->setXYTheta(x, y, theta );
}

void ControllerSceneManager::update()
{
    short renderAreaWidth = BCIService::getInstance()->bciRenderArea->getSize()[0];
    short renderAreaHeight = BCIService::getInstance()->bciRenderArea->getSize()[1];

    if(this->try_lock())
    {
        temp_targets = targets;
        targets.clear();
        //cursor->update(state, renderAreaWidth, renderAreaHeight);
        for(int i = 0; i < temp_targets.size(); i++)
        {
            if (temp_targets[i]->valid)
            {
                temp_targets[i]->update(state, renderAreaWidth, renderAreaHeight);
                if (temp_targets[i]->active)
                {
                    // temp_targets[i]->update2()
                }
                targets.push_back(temp_targets[i]);
            }
            else
            {
                SoDB::writelock();
                control_scene_separator->removeChild(temp_targets[i]->sprite_root);
                SoDB::writeunlock();
            }
        }

        // update target button on "next button" action
        if(this->state==CursorState::MOVING_SLOW)
        {
            std::cout << "temp_targets.size(): " << temp_targets.size() << std::endl;
            if (temp_targets.size() ==0)
            {
                this->unlock();
                return;
            }
            temp_targets[next_target]->active=false;
            temp_targets[next_target]->update2(renderAreaWidth, renderAreaHeight);
            next_target=next_target<temp_targets.size()-1 ? next_target+1:0;
            while(!(temp_targets[next_target]->valid))
            {
                next_target=next_target<temp_targets.size()-1 ? next_target+1:0;
            }
            temp_targets[next_target]->active=true;
            temp_targets[next_target]->update2(renderAreaWidth, renderAreaHeight);
            std::cout<<"<"<<next_target<<">"<<std::endl;
        }
        else if(this->state==CursorState::MOVING_FAST)
        {
            if (temp_targets.size() ==0)
            {
                this->unlock();
                return;
            }
            temp_targets[next_target]->setHit();
        }
        this->state=CursorState::SPINNING;
        this->unlock();
    }
}

void ControllerSceneManager::handleMouseButtonEvent(void *, SoEventCallback *eventCB)
{
    //    std::cout  << "handleMouseButtonEvent" << std::endl;

    const SoEvent *event = eventCB->getEvent();
    if (SO_MOUSE_RELEASE_EVENT(event,BUTTON1))
    {

        //      std::cout  << "X: " << event->getPosition().getValue()[0] << std::endl;
        //      std::cout  << "Y: " << event->getPosition().getValue()[1] << std::endl;

        short renderAreaWidth = BCIService::getInstance()->bciRenderArea->getSize()[0];
        short renderAreaHeight = BCIService::getInstance()->bciRenderArea->getSize()[1];

        //      std::cout  << "renderAreaWidth: " << renderAreaWidth << std::endl;
        //      std::cout  << "renderAreaHeight: " << renderAreaHeight << std::endl;
        //this works for full screen.
        double x = event->getPosition().getValue()[0]/500.0 - renderAreaWidth/1000.0;
        double y = event->getPosition().getValue()[1]/500.0 - renderAreaHeight/1000.0;

        //      double x = -renderAreaWidth/1000.0;
        //      double y = 0;

        //      std::cout  << "x: " << x << std::endl;
        //      std::cout  << "y: " << y << std::endl;

        // current_control_scene_manager->setCursorPosition(x, y, 0);
    }

}

void ControllerSceneManager::setState(int _state)
{
    state = _state;
}
