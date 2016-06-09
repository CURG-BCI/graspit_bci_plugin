#ifndef GRASPPLANNINGVIEW_H
#define GRASPPLANNINGVIEW_H

#include <QWidget>

namespace Ui {
class GraspPlanningView;
}

class GraspPlanningView : public QWidget
{
    Q_OBJECT
    
public:
    explicit GraspPlanningView(QWidget *parent = 0);
    ~GraspPlanningView();
    
private:
    Ui::GraspPlanningView *ui;
};

#endif // GRASPPLANNINGVIEW_H
