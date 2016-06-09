#include "graspPlanningView.h"
#include "ui_graspPlanningView.h"

GraspPlanningView::GraspPlanningView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GraspPlanningView)
{
    ui->setupUi(this);
}

GraspPlanningView::~GraspPlanningView()
{
    delete ui;
}
