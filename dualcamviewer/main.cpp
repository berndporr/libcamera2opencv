#include "window.h"

#include <QApplication>

// Main program
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	// create the window
	Window window;
	window.show();

	Libcam2OpenCV camera1;
	camera1.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							{ window.updateImage1(mat); });

	Libcam2OpenCV camera2;
	camera2.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							{ window.updateImage2(mat); });

	Libcam2OpenCVSettings settings;
	settings.cameraIndex = 0;
	camera1.start(settings);
	settings.cameraIndex = 1;
	camera2.start(settings);

	// execute the application
	const int r = app.exec();

	camera1.stop();
	camera2.stop();
	return r;
}
