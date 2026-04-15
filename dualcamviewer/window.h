#ifndef WINDOW_H
#define WINDOW_H

#include <qwt/qwt_thermo.h>

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>

#include "libcam2opencv.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    void updateImage1(const cv::Mat &mat);
    void updateImage2(const cv::Mat &mat);

private:
    QHBoxLayout  *hLayout;
    QLabel       *image1;
    QLabel       *image2;
 };

#endif // WINDOW_H
