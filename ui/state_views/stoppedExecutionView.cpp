#include "stoppedExecutionView.h"
#include "graspit_bci_plugin/ui_stoppedExecutionView.h"
#include "BCI/bciService.h"

StoppedExecutionView::StoppedExecutionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StoppedExecutionView)
{
    ui->setupUi(this);
}

StoppedExecutionView::~StoppedExecutionView()
{
    delete ui;
}


