#include "window.h"

#include <QApplication>

// Main program
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	libcamera::CameraManager cm;
	cm.start();

	// create the window
	Window window;
	window.show();

	Libcam2OpenCV camera;
	camera.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							{ window.updateImage(mat); });

	camera.start(cm);

	// execute the application
	const int r = app.exec();

	camera.stop();
	cm.stop();
	return r;
}
