#include "window.h"

Window::Window()
{
	image = new QLabel;
	vLayout = new QVBoxLayout();
	vLayout->addWidget(image);
	setLayout(vLayout);
}

void Window::updateImage(const cv::Mat &mat) {
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
			   QImage::Format_BGR888);
	image->setPixmap(QPixmap::fromImage(frame));
	update();
}
