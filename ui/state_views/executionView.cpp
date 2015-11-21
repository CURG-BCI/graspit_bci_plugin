#include "executionView.h"
#include "graspit_bci_plugin/ui_executionView.h"
#include "BCI/bciService.h"

ExecutionView::ExecutionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ExecutionView)
{
    ui->setupUi(this);
}

ExecutionView::~ExecutionView()
{
    delete ui;
}

