#pragma once
//�����Ƿ񱣴�ͼƬ
#define saveImages 1
//�����Ƿ��¼��Ƶ
#define recordVideo 1


#include <QtWidgets/QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QImage>
#include <QPixmap>
#include <QScreen>
#include "ui_QtGuiApplication.h"
#include <Qlabel>  
#include <qaction.h>
#include <QTimer>
#include <QPaintEvent>
#include <QInputDialog>
// ����OpenCV API
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>

//����PYLON API.
#include <pylon/PylonIncludes.h>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "math.h"


#ifdef PYLON_WIN_BUILD
#include <pylon/PylonGUI.h>    
//#include <stereo_calib.h>
#endif

//�����ռ�.
using namespace Pylon;
using namespace cv;
using namespace std;


class QtGuiApplication : public QMainWindow
{
	Q_OBJECT

public:
	QtGuiApplication(QWidget *parent = Q_NULLPTR);
	//ͼ����ʾ
	void showimage();  //��
	void showimage2();  //��
	void displayMat(QLabel *label, Mat image);   //��ʾ��label
	void saveL(Mat image);  //����
	void saveR(Mat image);  //����
	//����Ȥ����ѡȡ
	void on_MouseL(int event, int x, int y);
	static void onMouseL(int event, int x, int y, int, void* userdata);
	void on_MouseR(int event, int x, int y);
	static void onMouseR(int event, int x, int y, int, void* userdata);
	//ֱ������
	void squence(vector<cv::Point2f> &points);   //����
	void calcPoint(double A, double B, double C, double f, double a, double b, double c, double x0, double y0, int count, Point2f &Point);  //���Ӧ��
	void calculateWorldPoint(vector<cv::Point3f> &calcWorldPoint, vector<cv::Point2f> &Lpoints, vector<cv::Point2f> &Rpoints,
		Mat &l_cameraMatrix, Mat &l_disCoeff, Mat &r_cameraMatrix, Mat &r_disCoeff, Mat &R, Mat &T);//����������
	double calcDiameter(vector<cv::Point3f> &calcWorldPoint, int &count);//��ֱ��
	//��Ȳ���
	void calculateR(double &k, double x0, double y0, double &A, double &B, double &C, vector<cv::Point2f> &Rpoints);//����ͼ������
	void calcwide(vector<cv::Point3f> &WorldPoint1, vector<cv::Point3f> &WorldPoint2, double &width);
	//�б����
	void fit(Mat image_bin, Mat &srcImage, int &num, vector<cv::Point2f> &point);//�����Բ
	void calcrect(vector<cv::Point3f> &calcWorldPoint, double &length, double &width);//����εĳ���
private:
	Ui::QtGuiApplicationClass ui;
	//��ʾͼ��
	static const uint32_t c_countOfImagesToGrab = 1;   //����ץȡ��ͼ����
	QTimer *timer;   //��ʱ��
	Mat openCvImageL;   //��ͼ��
	Mat openCvImageR;   //��ͼ��
	Mat openCvImageLtemp;
	Mat openCvImageRtemp;
	int ImgCountL = 1;
	int saveflagL = 0;   //����
	int ImgCountR = 1;
	int saveflagR = 0;
	int lopen = 0;      //�Ƿ��ͼ��
	int ropen = 0;
	//����Ȥ����ѡȡ
	bool draw = false;
	Point cursor;//��ʼ����   
	Rect rect;//���ROI�ľ��ο�
	double areaL, areaR;//ROI�����
	//ֱ������
	Point2f leftEpsPoint[4], rightEpsPoint[4], test[4];
	Mat M1, D1, M2, D2, R, T, F;

private slots:    //�����źź���  
	void openclicked();     //������ͷ
	void closeclicked();     //�ر�����ͷ
	void saveclicked();      //����ͼ��
	void leftclicked();      //����ͼ
	void rightclicked();      //����ͼ
	void calibclicked();     //����궨
	void diamclicked();     //ֱ������
	void rectclicked();     //���β���
	void droneclicked();     //���β���	
	void LROIclicked();      //��ͼ����Ȥ����ѡȡ
	void RROIclicked();      //��ͼ����Ȥ����ѡȡ
	void recoverclicked();    //��ԭԭͼ
	void timeout();
	void change();    //�����ʾ

};
