#include "QtGuiApplication.h"
#include <QtWidgets/QApplication>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QtGuiApplication w;
	w.showMaximized();

	return a.exec();
}
