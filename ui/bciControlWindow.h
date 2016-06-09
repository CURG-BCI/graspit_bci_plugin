#ifndef BCICONTROLWINDOW_H
#define BCICONTROLWINDOW_H

#include <QDialog>
#include "ui_BCIControlWindowBase.h"
#include "BCI/bciService.h"
#include "include/debug.h"
#include "BCI/graspManager.h"
class BCIControlWindow: public QWidget, public Ui::BCIControlWindowBase
{

    Q_OBJECT

public:


    BCIControlWindow(QWidget *parent = 0 )
        :QWidget(parent)
    {
        BCIService::getInstance();
        if(GraspManager::getInstance()->thread() != this->thread())
            DBGA("OnlinePlannerController not in same thread as BCIControlWindow");


        setupUi(this);

        this->setBackgroundColor(QColor::fromRgb(112,128,144));


    }
public slots:
    void redraw()
    {
        this->bciWorldView->redraw();
    }
};


#endif // BCICONTROLWINDOW_H
