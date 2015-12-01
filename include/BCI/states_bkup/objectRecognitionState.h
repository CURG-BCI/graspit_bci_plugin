#ifndef OBJECTRECOGNITIONSTATE_H
#define OBJECTRECOGNITIONSTATE_H

#include "state_views/objectRecognitionView.h"
#include "bciControlWindow.h"
#include "BCI/state.h"

class ObjectRecognitionState : public State
{
        Q_OBJECT
public:
    ObjectRecognitionState(BCIControlWindow *_bciControlWindow,QState* parent=0);

    virtual void onEntry(QEvent *e);
    virtual void onExit(QEvent *e);

private:
   ObjectRecognitionView *objectRecognitionView;
   BCIControlWindow *bciControlWindow;
};

#endif // OBJECTRECOGNITIONSTATE_H
