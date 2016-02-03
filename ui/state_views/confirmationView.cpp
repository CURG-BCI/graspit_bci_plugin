#include "confirmationView.h"
#include "ui_confirmationView.h"
#include "BCI/bciService.h"
#include "include/graspitGUI.h"
#include "include/ivmgr.h"
#include "include/body.h"
#include "include/world.h"
#include "include/robot.h"
#include "include/grasp.h"
#include "BCI/onlinePlannerController.h"
#include "BCI/qtWaitingSpinner.h"
#include "BCI/handView.h"

ConfirmationView::ConfirmationView(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::ConfirmationView)
{
    ui->setupUi(this);

    SoQtExaminerViewer *mainViewer = graspItGUI->getIVmgr()->getViewer();
    Hand * h = OnlinePlannerController::getInstance()->getSolutionHand();
    QFrame *parentWindow = this->ui->previewFrame;
    QString viewName = QString("current best grasp");
    handView = new HandView(mainViewer,h,*parentWindow,viewName);

}


void ConfirmationView::setCurrentGrasp(Hand *hand, const GraspPlanningState *graspPlanningState)
{

    if(graspPlanningState)
    {
        handView->update(*graspPlanningState, *hand);
    }
}

ConfirmationView::~ConfirmationView()
{
    delete ui;
}

