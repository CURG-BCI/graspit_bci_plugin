#include "placementLocationSelectionView.h"
#include "graspit_bci_plugin/ui_placementLocationSelectionView.h"

PlacementLocationSelectionView::PlacementLocationSelectionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::PlacementLocationSelectionView)
{
    ui->setupUi(this);
}

PlacementLocationSelectionView::~PlacementLocationSelectionView()
{
    delete ui;
}
