#include "graspView.h"
#include "graspit_bci_plugin/ui_graspView.h"
#include "include/body.h"

GraspView::GraspView( QWidget *parent) :
    QWidget(parent),

    ui(new Ui::GraspView)
{
    ui->setupUi(this);
}

GraspView::~GraspView()
{
    delete ui;
}
