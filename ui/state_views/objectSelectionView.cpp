#include "objectSelectionView.h"
#include "ui_objectSelectionView.h"
#include "include/debug.h"

ObjectSelectionView::ObjectSelectionView(ObjectSelectionState *state,QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ObjectSelectionView)
{
    ui->setupUi(this);

}






ObjectSelectionView::~ObjectSelectionView()
{
    delete ui;
}
