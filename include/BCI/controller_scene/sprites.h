
#ifndef SPRITE_H
#define SPRITE_H


#include "qstring.h"
#include "qobject.h"
#include "QImage"

class SoAnnotation;
class SoImage;
class SoSFImage;
class SoTransform;


class Sprite: public QObject {

    Q_OBJECT ;

protected:

    double x;
    double y;
    double theta;

    static const std::string DEFAULT_SPRITE_DIR;

    QImage *qimage;
    SoImage *image;
    SoTransform * imageTran;
    QString filename;
    QString button_text;

    QString filename_1;
    QString filename_2;

    short renderAreaWidth;
    short renderAreaHeight;

    bool imageInitialized;

    void convert(const QImage& p, SoSFImage& img) const;
    void convert(const SoSFImage& p, QImage& img) const;

public:
    Sprite(SoAnnotation * control_scene_separator, QString filename, double x_, double y_, double theta_);
    virtual ~Sprite() {};
    virtual void update(int state, short renderAreaWidth, short renderAreaHeight)=0;
    bool intersects(QRectF *other_rect);

    bool valid;

    QRectF *bounding_rect;
    SoAnnotation *sprite_root;
};



class Target: public Sprite
{

    Q_OBJECT ;

protected:
    int steps_since_last_hit;

public:
//    Target(SoAnnotation * control_scene_separator, QString filename, double x_, double y_, double theta_, QString target_text);
    Target(SoAnnotation * control_scene_separator, QString filename, double x_, double y_, double theta_, QString target_text,
           QString inactive_filename="target_background.png", QString active_filename="target_active.png");

    virtual ~Target();
    void update(int state, short renderAreaWidth, short renderAreaHeight);
    void setHit();
        void update2(short renderAreaWidth_, short renderAreaHeight_);
    bool active;



signals:
        void hit();
};



class Cursor: public Sprite
{
        Q_OBJECT ;
public:
    Cursor(SoAnnotation * control_scene_separator, QString filename, double x_, double y_, double theta_);
    void setXYTheta(double x_, double y_, double theta_);
    void update(int state, short renderAreaWidth, short renderAreaHeight);

};

class Pipeline: public Sprite
{
        Q_OBJECT ;
public:
    Pipeline(SoAnnotation * control_scene_separator, QString filename, double x_, double y_, double theta_);
    //void setXYTheta(double x_, double y_, double theta_);
    ~Pipeline();
    void update(int state, short renderAreaWidth, short renderAreaHeight);


};
#endif
