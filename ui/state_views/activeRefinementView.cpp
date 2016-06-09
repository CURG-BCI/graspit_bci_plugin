#include "activeRefinementView.h"
#include "ui_activeRefinementView.h"
#include "BCI/bciService.h"
#include "BCI/graspManager.h"

using bci_experiment::GraspManager;

ActiveRefinementView::ActiveRefinementView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ActiveRefinementView)
{
    ui->setupUi(this);

    createHandView();
}



void ActiveRefinementView::createHandView()
{
    Hand * h = GraspManager::getInstance()->getHand();

    SoQtExaminerViewer *mainViewer = graspItGUI->getIVmgr()->getViewer();
    QString viewName = QString("current best grasp");
    QString nextViewName = QString("next grasp");
    currentGraspView = new HandView(mainViewer, h, *this->ui->previewFrame, viewName);
    nextGraspView = new HandView(mainViewer, h, *this->ui->previewNextFrame, nextViewName);
}

void ActiveRefinementView::showEvent(QShowEvent *)
{
    Hand * h = GraspManager::getInstance()->getHand();
    this->currentGraspView->updateGeom(*GraspManager::getInstance()->getHand());
    showSelectedGrasp(h,NULL);
    showNextGrasp(h,NULL);
}

void ActiveRefinementView::showSelectedGrasp(Hand *hand, const GraspPlanningState *graspPlanningState)
{
    if(graspPlanningState)
    {
        currentGraspView->update(*graspPlanningState, *hand);
    }
}

void ActiveRefinementView::showNextGrasp(Hand * hand,  const GraspPlanningState *graspPlanningState)
{
    if(graspPlanningState)
    {
        nextGraspView->update(*graspPlanningState, *hand);
    }
}

ActiveRefinementView::~ActiveRefinementView()
{
    delete ui;
}

