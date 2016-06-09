#include "graspSelectionView.h"
#include "ui_graspSelectionView.h"
#include "BCI/bciService.h"
#include "include/debug.h"

using bci_experiment::GraspManager;

GraspSelectionView::GraspSelectionView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::GraspSelectionView)
{
    ui->setupUi(this);

    SoQtExaminerViewer *mainViewer = graspItGUI->getIVmgr()->getViewer();
    Hand * h = GraspManager::getInstance()->getHand();
    QFrame *parentWindow = this->ui->renderArea;
    QString viewName = QString("current best grasp");
    selectedHandView = new HandView(mainViewer,h,*parentWindow, viewName);

    QFrame *nextParentWindow = this->ui->nextRenderArea;
    QString nextViewName = QString("next grasp");
    nextHandView = new HandView(mainViewer,h,*nextParentWindow, nextViewName);

}

void GraspSelectionView::showEvent(QShowEvent *)
{
    Hand * h = GraspManager::getInstance()->getHand();
    selectedHandView->updateGeom(*GraspManager::getInstance()->getHand());
    showSelectedGrasp(h,NULL);
    showNextGrasp(h, NULL);
}


void GraspSelectionView::showSelectedGrasp(Hand *hand ,const GraspPlanningState *graspPlanningState)
{

    if(graspPlanningState)
    {
        selectedHandView->update(*graspPlanningState, *hand);
    }
    else
    {
        DBGA("GraspSelectionView::showSelectedGrasp::No grasp");
    }
}

void GraspSelectionView::showNextGrasp(Hand *hand ,const GraspPlanningState *graspPlanningState)
{
    if(graspPlanningState)
    {
        nextHandView->update(*graspPlanningState, *hand);
    }
    else
    {
        DBGA("GraspSelectionView::showNextGrasp::No grasp");
    }
}


GraspSelectionView::~GraspSelectionView()
{
    delete ui;
}
