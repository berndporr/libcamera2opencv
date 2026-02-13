#include "window.h"

Window::Window()
{
	myCallback.window = this;
	camera.registerCallback(&myCallback);
	image = new QLabel;
	vLayout = new QVBoxLayout();
	vLayout->addWidget(image);
	setLayout(vLayout);
	camera.start();
}

Window::~Window()
{
	camera.stop();
}

void Window::updateImage(const cv::Mat &mat) {
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
			   QImage::Format_BGR888);
	image->setPixmap(QPixmap::fromImage(frame));
	update();
}
