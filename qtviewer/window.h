#ifndef WINDOW_H
#define WINDOW_H

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
    void updateImage(const cv::Mat &mat);

private:
    QVBoxLayout  *vLayout;  // vert layout
    QLabel       *image;
 };

#endif // WINDOW_H
