#include "window.h"

Window::Window()
{
	image1 = new QLabel;
	image2 = new QLabel;
	hLayout = new QHBoxLayout();
	hLayout->addWidget(image1);
	hLayout->addWidget(image2);
	setLayout(hLayout);
}

void Window::updateImage1(const cv::Mat &mat) {
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
			   QImage::Format_BGR888);
	image1->setPixmap(QPixmap::fromImage(frame));
	update();
}

void Window::updateImage2(const cv::Mat &mat) {
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
			   QImage::Format_BGR888);
	image2->setPixmap(QPixmap::fromImage(frame));
	update();
}
