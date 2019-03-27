#pragma once
//定义是否保存图片
#define saveImages 1
//定义是否记录视频
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
// 加载OpenCV API
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/opencv.hpp>

//加载PYLON API.
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

//命名空间.
using namespace Pylon;
using namespace cv;
using namespace std;


class QtGuiApplication : public QMainWindow
{
	Q_OBJECT

public:
	QtGuiApplication(QWidget *parent = Q_NULLPTR);
	//图像显示
	void showimage();  //左
	void showimage2();  //右
	void displayMat(QLabel *label, Mat image);   //显示在label
	void saveL(Mat image);  //保存
	void saveR(Mat image);  //保存
	//感兴趣区域选取
	void on_MouseL(int event, int x, int y);
	static void onMouseL(int event, int x, int y, int, void* userdata);
	void on_MouseR(int event, int x, int y);
	static void onMouseR(int event, int x, int y, int, void* userdata);
	//直径测量
	void squence(vector<cv::Point2f> &points);   //排序
	void calcPoint(double A, double B, double C, double f, double a, double b, double c, double x0, double y0, int count, Point2f &Point);  //求对应点
	void calculateWorldPoint(vector<cv::Point3f> &calcWorldPoint, vector<cv::Point2f> &Lpoints, vector<cv::Point2f> &Rpoints,
		Mat &l_cameraMatrix, Mat &l_disCoeff, Mat &r_cameraMatrix, Mat &r_disCoeff, Mat &R, Mat &T);//求世界坐标
	double calcDiameter(vector<cv::Point3f> &calcWorldPoint, int &count);//求直径
	//宽度测量
	void calculateR(double &k, double x0, double y0, double &A, double &B, double &C, vector<cv::Point2f> &Rpoints);//找右图特征点
	void calcwide(vector<cv::Point3f> &WorldPoint1, vector<cv::Point3f> &WorldPoint2, double &width);
	//靶标测量
	void fit(Mat image_bin, Mat &srcImage, int &num, vector<cv::Point2f> &point);//拟合椭圆
	void calcrect(vector<cv::Point3f> &calcWorldPoint, double &length, double &width);//求矩形的长宽
private:
	Ui::QtGuiApplicationClass ui;
	//显示图像
	static const uint32_t c_countOfImagesToGrab = 1;   //定义抓取的图像数
	QTimer *timer;   //定时器
	Mat openCvImageL;   //左图像
	Mat openCvImageR;   //右图像
	Mat openCvImageLtemp;
	Mat openCvImageRtemp;
	int ImgCountL = 1;
	int saveflagL = 0;   //保存
	int ImgCountR = 1;
	int saveflagR = 0;
	int lopen = 0;      //是否打开图像
	int ropen = 0;
	//感兴趣区域选取
	bool draw = false;
	Point cursor;//初始坐标   
	Rect rect;//标记ROI的矩形框
	double areaL, areaR;//ROI的面积
	//直径测量
	Point2f leftEpsPoint[4], rightEpsPoint[4], test[4];
	Mat M1, D1, M2, D2, R, T, F;

private slots:    //声明信号函数  
	void openclicked();     //打开摄像头
	void closeclicked();     //关闭摄像头
	void saveclicked();      //保存图像
	void leftclicked();      //打开左图
	void rightclicked();      //打开左图
	void calibclicked();     //相机标定
	void diamclicked();     //直径测量
	void rectclicked();     //矩形测量
	void droneclicked();     //矩形测量	
	void LROIclicked();      //左图感兴趣区域选取
	void RROIclicked();      //右图感兴趣区域选取
	void recoverclicked();    //还原原图
	void timeout();
	void change();    //结果显示

};
