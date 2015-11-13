

#include <QWidget>

#include<Inventor/Qt/SoQtRenderArea.h>
#include<Inventor/nodes/SoSeparator.h>
#include<Inventor/nodes/SoMaterial.h>
#include<Inventor/nodes/SoTransformSeparator.h>

#include <Inventor/nodes/SoOrthographicCamera.h>
#include <Inventor/nodes/SoPerspectiveCamera.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include<Inventor/nodes/SoLightModel.h>

#include "include/graspitGUI.h"
#include "include/ivmgr.h"


class BCIWidgetManager
{

public:
    BCIWidgetManager();
    ~BCIWidgetManager();

private:
    SoQtRenderArea * renderArea;
};

