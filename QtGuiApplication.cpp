#include "QtGuiApplication.h"

QtGuiApplication::QtGuiApplication(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	timer = new QTimer(this);
	ui.open_camera->setEnabled(true);
	ui.close_camera->setEnabled(false);
	ui.save_camera->setEnabled(false);
	ui.diam->setEnabled(false);
	ui.rect->setEnabled(false);
	ui.drone->setEnabled(false);
	ui.LROI->setEnabled(false);
	ui.RROI->setEnabled(false);
	ui.recover->setEnabled(false);
	connect(timer, SIGNAL(timeout()), this, SLOT(timeout()));

}
/*----------------------------------------------------定时开启摄像头----------------------------------------------------*/
void QtGuiApplication::timeout()
{
	showimage();
	showimage2();
}
/*----------------------------------------------------开启摄像头----------------------------------------------------*/
void QtGuiApplication::openclicked()
{
	saveflagL = 0;
	saveflagR = 0;
	timer->start(1000);
	ui.open_camera->setEnabled(false);
	ui.close_camera->setEnabled(true);
	ui.save_camera->setEnabled(true);
	ui.open_left->setEnabled(false);
	ui.open_right->setEnabled(false);
}
/*----------------------------------------------------关闭摄像头----------------------------------------------------*/
void QtGuiApplication::closeclicked()
{
	timer->stop();
	saveflagL = 0;
	saveflagR = 0;
	ui.open_camera->setEnabled(true);
	ui.close_camera->setEnabled(false);
	ui.save_camera->setEnabled(false);
	ui.open_left->setEnabled(true);
	ui.open_right->setEnabled(true);
	ui.label->clear();
	ui.label2->clear();
}
/*----------------------------------------------------保存图片----------------------------------------------------*/
void QtGuiApplication::saveclicked()
{
	saveflagL = 1;
	saveflagR = 1;
}
/*----------------------------------------------------打开左图----------------------------------------------------*/
void QtGuiApplication::leftclicked()
{
	//读取参数
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs["F"] >> F;
	fs.release();

	ui.label->clear();//先清除label内容    
	//打开图片操作    
	QString filename = QFileDialog::getOpenFileName(this, tr("left image"), "", tr("Images (*.png *.bmp *.jpg)"));
	if (filename.isEmpty())
		return;
	else
	{
		string file = filename.toStdString();
		openCvImageL = imread(file);
		openCvImageLtemp = imread(file);

		QImage img;
		if (!(img.load(filename))) //加载图像        
		{
			QMessageBox::information(this, tr("打开图像失败"), tr("打开图像失败!"));
			return;
		}
		ui.label->setPixmap(QPixmap::fromImage(img.scaled(ui.label->size())));
		lopen = 1;
		QString information = information.fromLocal8Bit("%1*%2").arg(openCvImageL.cols).arg(openCvImageL.rows);
		ui.lsize->setText(information);
		areaL = openCvImageL.cols*openCvImageL.rows;
	}
	if (lopen == ropen == 1)
	{
		ui.diam->setEnabled(true);
		ui.rect->setEnabled(true);
		ui.drone->setEnabled(true);
		ui.LROI->setEnabled(true);
		ui.RROI->setEnabled(true);
		ui.recover->setEnabled(true);
	}

}
/*----------------------------------------------------打开右图----------------------------------------------------*/
void QtGuiApplication::rightclicked()
{
	//读取参数
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs["F"] >> F;
	fs.release();

	ui.label2->clear();//先清除label内容    
	//打开图片操作    
	QString filename = QFileDialog::getOpenFileName(this, tr("right image"), "", tr("Images (*.png *.bmp *.jpg)"));
	if (filename.isEmpty())
		return;
	else
	{
		string file = filename.toStdString();
		openCvImageR = imread(file);
		openCvImageRtemp = imread(file);


		QImage img;
		if (!(img.load(filename))) //加载图像        
		{
			QMessageBox::information(this, tr("打开图像失败"), tr("打开图像失败!"));
			return;
		}
		ui.label2->setPixmap(QPixmap::fromImage(img.scaled(ui.label2->size())));
		ropen = 1;
		QString information = information.fromLocal8Bit("%1*%2").arg(openCvImageR.cols).arg(openCvImageR.rows);
		ui.rsize->setText(information);
		areaR = openCvImageR.cols*openCvImageR.rows;
	}
	if (lopen == ropen == 1)
	{
		ui.diam->setEnabled(true);
		ui.rect->setEnabled(true);
		ui.drone->setEnabled(true);
		ui.LROI->setEnabled(true);
		ui.RROI->setEnabled(true);
		ui.recover->setEnabled(true);
	}
}

/*----------------------------------------------------图像标定----------------------------------------------------*/
void QtGuiApplication::calibclicked()
{
	ui.resultshow->clear();
	ui.resultshow->textCursor().insertText("Start Calibration!\n");
	//提取角点			
	Size image_size;  // 图像的尺寸	
	QString row = ui.rownum->text();
	int rownum = row.toInt();
	QString column = ui.columnnum->text();
	int columnnum = column.toInt();
	Size board_size = Size(rownum, columnnum);    //标定板上每行、列的角点数
	QString space = ui.space->text();
	int sp = space.toInt();
	Size square_size = Size(sp, sp);  //圆心距


	QInputDialog dia(this);
	dia.setWindowTitle("Board Size");
	dia.setLabelText("Please input board_size.row：");
	dia.setInputMode(QInputDialog::TextInput);

	vector<Point2f> cornerL;          //左摄像机某一照片角点坐标集合  
	vector<Point2f> cornerR;          //右摄像机某一照片角点坐标集合  
	vector<vector<Point2f>> imagePointL;                   //左摄像机所有照片角点的坐标集合  
	vector<vector<Point2f>> imagePointR;                   //右摄像机所有照片角点的坐标集合  
	QDir *dir = new QDir("./calib/");
	QStringList filter;
	filter << "*.jpg";
	dir->setNameFilters(filter);
	QFileInfoList fileInfoList = dir->entryInfoList(filter);
	int imagecount = fileInfoList.count() / 2;  //标定图片数量
	int count = 0;
	int total = 0;
	while (count < imagecount)
	{
		char filename[100];
		/*读取左边的图像*/
		sprintf_s(filename, "calib\\left%02d.jpg", count + 1);
		Mat ImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
		/*读取右边的图像*/
		sprintf_s(filename, "calib\\right%02d.jpg", count + 1);
		Mat ImageR = imread(filename, CV_LOAD_IMAGE_COLOR);
		//openCvImageL = imread(filename, CV_LOAD_IMAGE_COLOR);
		image_size.width = ImageL.cols;
		image_size.height = ImageL.rows;
		//提取角点		
		bool foundL = false;
		bool foundR = false;
		foundL = findCirclesGrid(ImageL, board_size, cornerL);
		foundR = findCirclesGrid(ImageR, board_size, cornerR);
		if (foundL == true && foundR == true)
		{
			imagePointL.push_back(cornerL); //保存亚像素角点
			imagePointR.push_back(cornerR);

			Mat ImageL_gray;
			Mat ImageR_gray;
			cvtColor(ImageL, ImageL_gray, CV_RGB2GRAY);
			cvtColor(ImageR, ImageR_gray, CV_RGB2GRAY);
			//显示角点位置
			drawChessboardCorners(ImageL, board_size, cornerL, foundL); //用于在图片中标记角点
			drawChessboardCorners(ImageR, board_size, cornerR, foundR);
			//imshow("ImageL", ImageL);//显示图片		
			//imshow("ImageR", ImageR);
			displayMat(ui.label, ImageL);
			displayMat(ui.label2, ImageR);
			//waitKey(10);
			total++;
		}

		count++;
	}

	vector<vector<Point3f>> object_points; //标定板上角点的三维坐标 	
	Mat cameraMatrix[2]; //摄像机内参数矩阵 
	Mat distCoeffs[2]; //摄像机的5个畸变系数：k1,k2,p1,p2,k3 
	vector<Mat> tvecsMat;  // 每幅图像的旋转向量/
	vector<Mat> rvecsMat; //每幅图像的平移向量/
	/* 初始化标定板上角点的三维坐标 */
	for (int t = 0; t < total; t++)
	{
		vector<Point3f> tempPointSet;
		for (int i = 0; i < board_size.height; i++)
		{
			for (int j = 0; j < board_size.width; j++)
			{
				Point3f realPoint;
				realPoint.x = j * square_size.width;
				realPoint.y = i * square_size.height;
				realPoint.z = 0;
				tempPointSet.push_back(realPoint);
			}
		}
		object_points.push_back(tempPointSet);
	}

	double rmsL = calibrateCamera(object_points, imagePointL, image_size, cameraMatrix[0], distCoeffs[0], rvecsMat, tvecsMat, 0);
	double rmsR = calibrateCamera(object_points, imagePointR, image_size, cameraMatrix[1], distCoeffs[1], rvecsMat, tvecsMat, 0);
	Mat R, T, E, F;
	double rms = stereoCalibrate(object_points, imagePointL, imagePointR,
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		image_size, R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

	//误差写入文件
	ofstream outfile;
	outfile.open("error.txt");
	outfile << "rmsL=" << rmsL << "\n";
	outfile << "rmsR=" << rmsR << "\n";
	outfile << "rms=" << rms << "\n";
	QString informationL = informationL.fromLocal8Bit("rmsL = %1\n").arg(rmsL);
	QString informationR = informationR.fromLocal8Bit("rmsR = %1\n").arg(rmsR);
	QString information = information.fromLocal8Bit("rms = %1\n").arg(rms);
	ui.resultshow->textCursor().insertText(informationL);
	ui.resultshow->textCursor().insertText(informationR);
	ui.resultshow->textCursor().insertText(information);

	//保存内外参数
	/*
	Mat fundamental_matrix = Mat(3, 3, CV_32F);   //基本矩阵
	int num = rownum * columnnum;   //靶标共有多少个圆
	Mat p1 = Mat(num, 2, CV_32F);
	Mat p2 = Mat(num, 2, CV_32F);
	for (int i = 0; i < num; i++)
	{
		p1.at<float>(i, 0) = cornerL[i].x;
		p1.at<float>(i, 1) = cornerL[i].y;
		p2.at<float>(i, 0) = cornerR[i].x;
		p2.at<float>(i, 1) = cornerR[i].y;
	}
	fundamental_matrix = findFundamentalMat(p1, p2, CV_FM_8POINT);
	//fundamental_matrix = findFundamentalMat(p1, p2, FM_RANSAC);
	*/
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1] <<
			"F" << F;
		fs.release();
	}

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T;
		fs.release();
	}
	ui.resultshow->textCursor().insertText("Calibration completion!\n");
}

/*----------------------------------------------------左图ROI提取----------------------------------------------------*/
void QtGuiApplication::LROIclicked()
{
	ui.resultshow->clear();
	Mat oriImageL;
	openCvImageL.copyTo(oriImageL);
	namedWindow("LROI");
	imshow("LROI", oriImageL);

	setMouseCallback("LROI", QtGuiApplication::onMouseL, this);

}
void QtGuiApplication::onMouseL(int event, int x, int y, int, void* userdata)

{
	QtGuiApplication* temp = reinterpret_cast<QtGuiApplication*>(userdata);
	temp->on_MouseL(event, x, y);
}
void QtGuiApplication::on_MouseL(int event, int x, int y)
{
	Mat roi, mask;//ROI图像
	Mat  img = openCvImageL.clone();;
	switch (event)
	{
		//按下鼠标左键	
	case CV_EVENT_LBUTTONDOWN:
		//存放起始坐标  		
		cursor = Point(x, y);
		//初始化起始矩形框  		
		rect = Rect(x, y, 0, 0);
		draw = true;
		break;
		//松开鼠标左键      	
	case CV_EVENT_LBUTTONUP:
		if (rect.height > 0 && rect.width > 0)
		{
			//将img中的矩形区域复制给roi，并显示在窗口 			
			roi = img(Rect(rect.x, rect.y, rect.width, rect.height));
			rectangle(img, rect, Scalar(0, 0, 255), 2);
			imshow("LROI", img);
			mask = Mat::zeros(img.size(), CV_8UC1);
			mask(rect).setTo(255);
			Mat image = openCvImageLtemp = openCvImageL.clone();
			if (openCvImageL.channels() == 3)
			{
				cvtColor(openCvImageL, image, CV_BGR2GRAY);
			}
			image.copyTo(openCvImageLtemp, mask);
			cvtColor(openCvImageLtemp, openCvImageLtemp, CV_GRAY2RGB);
			areaL = rect.width* rect.height;
			displayMat(ui.label, openCvImageLtemp);
		}
		draw = false;
		break;
		//移动光标	
	case CV_EVENT_MOUSEMOVE:
		if (draw)
		{
			//用MIN得到左上点作为矩形框的起始坐标，如果不加这个，画矩形时只能向一个方向进行  			
			rect.x = MIN(x, cursor.x);
			rect.y = MIN(y, cursor.y);
			rect.width = abs(cursor.x - x);
			rect.height = abs(cursor.y - y);
			//防止矩形区域超出图像的范围  			
			rect &= Rect(0, 0, img.cols, img.rows);
		}
		break;
	}
}

/*----------------------------------------------------右图ROI提取----------------------------------------------------*/
void QtGuiApplication::RROIclicked()
{
	ui.resultshow->clear();
	Mat oriImageR;
	openCvImageR.copyTo(oriImageR);
	namedWindow("RROI");
	imshow("RROI", oriImageR);

	setMouseCallback("RROI", QtGuiApplication::onMouseR, this);

}
void QtGuiApplication::onMouseR(int event, int x, int y, int, void* userdata)

{
	QtGuiApplication* temp = reinterpret_cast<QtGuiApplication*>(userdata);
	temp->on_MouseR(event, x, y);
}
void QtGuiApplication::on_MouseR(int event, int x, int y)
{
	Mat roi, mask;//ROI图像
	Mat  img = openCvImageR.clone();
	switch (event)
	{
		//按下鼠标左键	
	case CV_EVENT_LBUTTONDOWN:
		//存放起始坐标  		
		cursor = Point(x, y);
		//初始化起始矩形框  		
		rect = Rect(x, y, 0, 0);
		draw = true;
		break;
		//松开鼠标左键      	
	case CV_EVENT_LBUTTONUP:
		if (rect.height > 0 && rect.width > 0)
		{
			//将img中的矩形区域复制给roi，并显示在窗口 			
			roi = img(Rect(rect.x, rect.y, rect.width, rect.height));
			rectangle(img, rect, Scalar(0, 0, 255), 2);
			imshow("RROI", img);
			mask = Mat::zeros(img.size(), CV_8UC1);
			mask(rect).setTo(255);
			Mat image = openCvImageRtemp = openCvImageR.clone();
			if (openCvImageR.channels() == 3)
			{
				cvtColor(openCvImageR, image, CV_BGR2GRAY);
			}
			image.copyTo(openCvImageRtemp, mask);
			cvtColor(openCvImageRtemp, openCvImageRtemp, CV_GRAY2RGB);
			areaR = rect.width*rect.height;
			displayMat(ui.label2, openCvImageRtemp);
		}
		draw = false;
		break;
		//移动光标	
	case CV_EVENT_MOUSEMOVE:
		if (draw)
		{
			//用MIN得到左上点作为矩形框的起始坐标，如果不加这个，画矩形时只能向一个方向进行  			
			rect.x = MIN(x, cursor.x);
			rect.y = MIN(y, cursor.y);
			rect.width = abs(cursor.x - x);
			rect.height = abs(cursor.y - y);
			//防止矩形区域超出图像的范围  			
			rect &= Rect(0, 0, img.cols, img.rows);
		}
		break;
	}
}

/*----------------------------------------------------还原原图----------------------------------------------------*/
void QtGuiApplication::recoverclicked()
{
	openCvImageL.copyTo(openCvImageLtemp);
	openCvImageR.copyTo(openCvImageRtemp);
	areaL = openCvImageLtemp.cols*openCvImageLtemp.rows;
	areaR = openCvImageRtemp.cols*openCvImageRtemp.rows;
	displayMat(ui.label, openCvImageL);
	displayMat(ui.label2, openCvImageR);
}

/*----------------------------------------------------直径测量----------------------------------------------------*/
void QtGuiApplication::diamclicked()
{
	ui.resultshow->clear();
	ui.resultshow->textCursor().insertText("Start Measurement!\n");
	double Diameter;
	Point2f LeftO, RightO;

	//打开图像
	Mat oriImageL, oriImageR;
	openCvImageLtemp.copyTo(oriImageL);
	openCvImageRtemp.copyTo(oriImageR);
	//读取参数
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs["F"] >> F;
	fs.release();
	fs.open("extrinsics.yml", FileStorage::READ);
	fs["R"] >> R;
	fs["T"] >> T;

	Mat image_binL, image_binR;
	Mat grayImageL, grayImageR;
	cvtColor(oriImageL, grayImageL, CV_BGR2GRAY);
	cvtColor(oriImageR, grayImageR, CV_BGR2GRAY);
	Mat dstImageL, dstImageR;
	dstImageL.create(grayImageL.size(), grayImageL.type());
	dstImageR.create(grayImageR.size(), grayImageR.type());

	//提取边缘
	GaussianBlur(grayImageL, dstImageL, Size(5, 5), 0, 0);
	//medianBlur(grayImageL, dstImageL, 5);
	//equalizeHist(dstImageL, dstImageL);
	Canny(dstImageL, image_binL, 220, 240, 3);
	//threshold(dstImageL, image_binL, 130, 255, THRESH_BINARY);
	//imshow("erzhihuaL", image_binL);
	//waitKey(30);

	GaussianBlur(grayImageR, dstImageR, Size(5, 5), 0, 0);
	//medianBlur(grayImageR, dstImageR, 5);
	//blur(grayImageR, dstImageR, Size(3, 3));
	//equalizeHist(dstImageR, dstImageR);
	Canny(dstImageR, image_binR, 200, 240, 3);
	//threshold(dstImageR, image_binR, 100, 255, THRESH_BINARY);
	//imshow("erzhihuaR", image_binR);
	//waitKey(30);

	//左图拟合
	Mat ImageL = Mat::zeros(image_binL.size(), CV_8UC1);
	vector<vector<cv::Point>> contoursLtemp;
	vector<Vec4i> hierarchy;
	cv::findContours(image_binL, contoursLtemp, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_NONE);


	for (int i = 0; i < contoursLtemp.size(); i++)
	{
		Mat pointsf;
		if (contoursLtemp[i].size() < 150)
			continue;
		if (abs(contourArea(contoursLtemp[i]) - areaL) / areaL < 0.01)     //去掉ROI边框
			continue;
		//将轮廓中的点转换为以Mat形式存储的2维点集(x,y)
		Mat(contoursLtemp[i]).convertTo(pointsf, CV_32F);
		RotatedRect box = fitEllipse(pointsf);
		double S1 = contourArea(contoursLtemp[i]);
		double S2 = box.size.height*box.size.width / 4 * CV_PI;
		//if ((S1 / S2 < 0.9) || (S1 / S2 > 1.1))
			//continue;
		drawContours(ImageL, contoursLtemp, i, Scalar(255), 1, 8, hierarchy);
	}

	vector<vector<cv::Point>> contoursL;
	cv::findContours(ImageL, contoursL, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	Mat pointsfL;
	Mat(contoursL[0]).convertTo(pointsfL, CV_32F);
	RotatedRect left_Eps = fitEllipse(pointsfL);

	//右图拟合
	Mat ImageR = Mat::zeros(image_binR.size(), CV_8UC1);
	vector<vector<cv::Point>> contoursRtemp;
	cv::findContours(image_binR, contoursRtemp, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_NONE);
	for (int i = 0; i < contoursRtemp.size(); i++)
	{
		Mat pointsf;
		if (contoursRtemp[i].size() < 150)
			continue;
		if (abs(contourArea(contoursRtemp[i]) - areaR) / areaR < 0.01)     //去掉ROI边框
			continue;
		//将轮廓中的点转换为以Mat形式存储的2维点集(x,y)
		Mat(contoursRtemp[i]).convertTo(pointsf, CV_32F);
		RotatedRect box = fitEllipse(pointsf);
		double S3 = contourArea(contoursRtemp[i]);
		double S4 = box.size.height*box.size.width / 4 * CV_PI;
		//if ((S3 / S4 < 0.9) || (S3 / S4 > 1.1))
			//continue;
		drawContours(ImageR, contoursRtemp, i, Scalar(255), 1, 8, hierarchy);
	}

	vector<vector<cv::Point>> contoursR;
	cv::findContours(ImageR, contoursR, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	Mat pointsfR;
	Mat(contoursR[0]).convertTo(pointsfR, CV_32F);
	RotatedRect right_Eps = fitEllipse(pointsfR);


	LeftO = left_Eps.center;
	RightO = right_Eps.center;
	Point2f LeftVertex[4], RightVertex[4];
	left_Eps.points(LeftVertex);
	right_Eps.points(RightVertex);
	float ltheta = left_Eps.angle * CV_PI / 180.0;
	float la = left_Eps.size.width / 2.0;
	float lb = left_Eps.size.height / 2.0;
	float rtheta = right_Eps.angle* CV_PI / 180.0;
	float ra = right_Eps.size.width / 2.0;
	float rb = right_Eps.size.height / 2.0;

	double Left_A = la * la * sin(ltheta) * sin(ltheta) + lb * lb * cos(ltheta) * cos(ltheta);
	double Left_B = (-2.0) * (la * la - lb * lb) * sin(ltheta) * cos(ltheta);
	double Left_C = la * la * cos(ltheta) * cos(ltheta) + lb * lb * sin(ltheta) * sin(ltheta);
	double Left_F = (-1.0) * la * la * lb * lb;

	double Right_A = ra * ra * sin(rtheta) * sin(rtheta) + rb * rb * cos(rtheta) * cos(rtheta);
	double Right_B = (-2.0) * (ra * ra - rb * rb) * sin(rtheta) * cos(rtheta);
	double Right_C = ra * ra * cos(rtheta) * cos(rtheta) + rb * rb * sin(rtheta) * sin(rtheta);
	double Right_F = (-1.0) * ra * ra * rb * rb;
	//画出拟合椭圆
	//Mat drawingL = Mat::zeros(oriImageL.size(), CV_8UC3);
	//Mat drawingR = Mat::zeros(oriImageR.size(), CV_8UC3);
	Mat drawingL, drawingR;
	openCvImageLtemp.copyTo(drawingL);
	openCvImageRtemp.copyTo(drawingR);

	Scalar color = Scalar(255, 0, 0);
	ellipse(drawingL, left_Eps, color, 2, 8);
	ellipse(drawingR, right_Eps, color, 2, 8);

	vector<Point2f> LPoint, RPoint;   //左右特征点
	//计算左右图特征点，以及对应极线
	for (int i = 0; i < 4; i++)
	{
		if (i == 3)
		{
			leftEpsPoint[i].x = (LeftVertex[i].x + LeftVertex[0].x) / 2;
			leftEpsPoint[i].y = (LeftVertex[i].y + LeftVertex[0].y) / 2;
			LPoint.push_back(leftEpsPoint[i]);
			rightEpsPoint[i].x = (RightVertex[i].x + RightVertex[0].x) / 2;
			rightEpsPoint[i].y = (RightVertex[i].y + RightVertex[0].y) / 2;
			RPoint.push_back(rightEpsPoint[i]);
			break;
		}
		leftEpsPoint[i].x = (LeftVertex[i].x + LeftVertex[i + 1].x) / 2;
		leftEpsPoint[i].y = (LeftVertex[i].y + LeftVertex[i + 1].y) / 2;
		LPoint.push_back(leftEpsPoint[i]);
		rightEpsPoint[i].x = (RightVertex[i].x + RightVertex[i + 1].x) / 2;
		rightEpsPoint[i].y = (RightVertex[i].y + RightVertex[i + 1].y) / 2;
		RPoint.push_back(rightEpsPoint[i]);
	}

	//画左图特征点
	for (int i = 0; i < 4; i++)
	{
		//circle(drawingL, leftEpsPoint[i], 3, Scalar(0, 255, 0), 5);
	}
	displayMat(ui.label, drawingL);

	//4点排序：y最大，y最小，x最大，x最小
	squence(LPoint);
	squence(RPoint);
	stringstream l;
	l << "LPoints:" << LPoint[0] << "," << LPoint[1] << "," << LPoint[2] << "," << LPoint[3] << endl;
	string lpoint = l.str();
	QString lpoints = QString::fromStdString(lpoint);
	ui.resultshow->textCursor().insertText(lpoints);

	for (int i = 0; i < 4; i++)
	{
		test[i] = RPoint[i];
	}
	double linea[4], lineb[4], linec[4];

	//计算右图椭圆方程与对应极线的交点作为右图对应特征点
	vector<Vec3f> epilines1;
	computeCorrespondEpilines(LPoint, 1, F, epilines1);

	for (int i = 0; i < 4; i++)
	{
		linea[i] = epilines1[i][0];
		lineb[i] = epilines1[i][1];
		linec[i] = epilines1[i][2];
	}

	for (int i = 0; i < 4; i++)
	{
		lineb[i] = lineb[i] / linea[i];
		linec[i] = linec[i] / linea[i];
		linea[i] = 1;
	}
	Right_B = Right_B / Right_A;
	Right_C = Right_C / Right_A;
	Right_F = Right_F / Right_A;
	Right_A = 1;

	for (int i = 0; i < 4; i++)
	{
		int m = i;
		calcPoint(Right_A, Right_B, Right_C, Right_F, linea[i], lineb[i], linec[i], RightO.x, RightO.y, m, rightEpsPoint[i]);
	}

	//画右图特征点和右极线
	for (int i = 0; i < 4; i++)
	{
		if (rightEpsPoint[i].x > 0 && rightEpsPoint[i].y > 0)
		{
			//circle(drawingR, rightEpsPoint[i], 3, Scalar(0, 255, 0), 5);
			//line(drawingR, Point(100, (-linea[i] * 100 - linec[i]) / lineb[i]),
				//Point(1500, (-linea[i] * 1500 - linec[i]) / lineb[i]), Scalar(0, 0, 255), 2, CV_AA);
		}
	}

	displayMat(ui.label2, drawingR);
	//整理左右图的特征点
	vector<Point3f>  WorldPoint;
	vector<Point2f> LPointtemp, RPointtemp;
	for (int i = 0; i < 4; i++)
	{
		LPointtemp.push_back(LPoint[i]);
	}
	LPoint.clear();
	RPoint.clear();
	int count = 0;
	for (int i = 0; i < 4; i++)
	{
		RPoint.push_back(rightEpsPoint[i]);
		LPoint.push_back(LPointtemp[i]);
		count++;
	}
	LPointtemp.clear();

	//反算左图特征点
	vector<Vec3f> epilines2;
	computeCorrespondEpilines(RPoint, 2, F, epilines2);
	for (int i = 0; i < 4; i++)
	{
		linea[i] = epilines1[i][0];
		lineb[i] = epilines1[i][1];
		linec[i] = epilines1[i][2];
	}

	for (int i = 0; i < 4; i++)
	{
		lineb[i] = lineb[i] / linea[i];
		linec[i] = linec[i] / linea[i];
		linea[i] = 1;
	}

	Left_B = Left_B / Left_A;
	Left_C = Left_C / Left_A;
	Left_F = Left_F / Left_A;
	Left_A = 1;
	for (int i = 0; i < 4; i++)
	{
		calcPoint(Left_A, Left_B, Left_C, Left_F, linea[i], lineb[i], linec[i], LeftO.x, LeftO.y, i, leftEpsPoint[i]);
	}
	//比较左图的特征点
	int count1 = 0;
	for (int i = 0; i < 4; i++)
	{
		if (leftEpsPoint[i].x > 0 && leftEpsPoint[i].y > 0)
		{
			if (abs(leftEpsPoint[i].x - LPoint[i].x) < 50 && abs(leftEpsPoint[i].y - LPoint[i].y) < 50)
			{
				LPointtemp.push_back(LPoint[i]);
				RPointtemp.push_back(RPoint[i]);
				count1++;
			}
		}
	}
	LPointtemp.push_back(LeftO);
	RPointtemp.push_back(RightO);
	//计算特征点与圆心点对应的三维坐标
	calculateWorldPoint(WorldPoint, LPointtemp, RPointtemp, M1, D1, M2, D2, R, T);


	//计算直径
	Diameter = calcDiameter(WorldPoint, count1);
	//结果显示
	QString information = information.fromLocal8Bit("直径为:%1mm\n").arg(Diameter);
	ui.resultshow->textCursor().insertText(information);
	ui.resultshow->textCursor().insertText("Measurement Completion!\n");
}

void QtGuiApplication::squence(vector<cv::Point2f> &points)
{
	Point2f tempPoint = points[0];
	Point2f temp[4];
	int i;
	for (i = 1; i < 4; i++)
	{
		if (points[i].x < tempPoint.x)
		{
			tempPoint.x = points[i].x;
			tempPoint.y = points[i].y;
		}
	}
	temp[3] = tempPoint;

	tempPoint = points[0];
	for (i = 1; i < 4; i++)
	{
		if (points[i].x >= tempPoint.x)
		{
			tempPoint.x = points[i].x;
			tempPoint.y = points[i].y;
		}
	}
	temp[2] = tempPoint;

	tempPoint = points[0];
	for (i = 1; i < 4; i++)
	{
		if (points[i].y < tempPoint.y)
		{
			tempPoint.x = points[i].x;
			tempPoint.y = points[i].y;
		}
	}
	temp[1] = tempPoint;

	tempPoint = points[0];
	for (i = 1; i < 4; i++)
	{
		if (points[i].y >= tempPoint.y)
		{
			tempPoint.x = points[i].x;
			tempPoint.y = points[i].y;
		}
	}
	temp[0] = tempPoint;

	for (i = 0; i < 4; i++)
	{
		points[i] = temp[i];
	}

}


void QtGuiApplication::calcPoint(double A, double B, double C, double f, double a, double b, double c, double x0, double y0, int count, Point2f &Point)
{

	double delta, x1, x2, y1, y2;
	double formula_A, formula_B, formula_C;
	formula_A = A - a / b * B + C * a*a / b / b;
	formula_B = -2 * A*x0 - c / b * B - y0 * B + a / b * x0*B + 2 * a / b * (c / b + y0)*C;
	formula_C = A * x0*x0 + (c / b * x0 + x0 * y0)*B + C * (c / b + y0)*(c / b + y0) + f;

	delta = formula_B * formula_B - 4 * formula_A*formula_C;

	if (delta == 0)
	{
		Point.x = -formula_B / 2 / formula_A;
		Point.y = -a * b*Point.x - c / b;
	}
	else if (delta < 0)
	{
		Point.x = 0;
		Point.y = 0;
	}
	else
	{
		x1 = (-formula_B + sqrt(delta)) / (2 * formula_A);
		x2 = (-formula_B - sqrt(delta)) / (2 * formula_A);
		y1 = -a / b * x1 - c / b;
		y2 = -a / b * x2 - c / b;
		switch (count)
		{
		case 0:
		{
			if (y1 >= y2)
			{
				Point.x = x1;
				Point.y = y1;
			}
			else
			{
				Point.x = x2;
				Point.y = y2;
			}
			break;
		}
		case 1:
		{
			if (y1 < y2)
			{
				Point.x = x1;
				Point.y = y1;
			}
			else
			{
				Point.x = x2;
				Point.y = y2;
			}
			break;
		}
		case 2:
		{
			if (x1 >= x2)
			{
				Point.x = x1;
				Point.y = y1;
			}
			else
			{
				Point.x = x2;
				Point.y = y2;
			}
			break;
		}
		case 3:
		{
			if (x1 < x2)
			{
				Point.x = x1;
				Point.y = y1;
			}
			else
			{
				Point.x = x2;
				Point.y = y2;
			}
			break;
		}
		default:
			break;
		}
	}
}

//求直径
double QtGuiApplication::calcDiameter(vector<cv::Point3f> &calcWorldPoint, int &count)
{
	double temp = 0;
	for (int i = 0; i < count; i++)
	{
		temp = temp + sqrt((calcWorldPoint[i].x - calcWorldPoint[count].x)*(calcWorldPoint[i].x - calcWorldPoint[count].x) +
			(calcWorldPoint[i].y - calcWorldPoint[count].y)*(calcWorldPoint[i].y - calcWorldPoint[count].y) +
			(calcWorldPoint[i].z - calcWorldPoint[count].z)*(calcWorldPoint[i].z - calcWorldPoint[count].z));
	}
	return temp / count * 2;
}

/*----------------------------------------------------宽度测量----------------------------------------------------*/
void QtGuiApplication::rectclicked()
{
	ui.resultshow->clear();
	ui.resultshow->textCursor().insertText("Start Measurement!\n");
	//打开图像
	Mat oriImageL, oriImageR;
	openCvImageLtemp.copyTo(oriImageL);
	openCvImageRtemp.copyTo(oriImageR);
	//读取参数
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs["F"] >> F;
	fs.release();
	fs.open("extrinsics.yml", FileStorage::READ);
	fs["R"] >> R;
	fs["T"] >> T;

	Mat  image_binL, image_binR;
	Mat grayImageL, grayImageR;
	cvtColor(oriImageL, grayImageL, CV_RGB2GRAY);
	cvtColor(oriImageR, grayImageR, CV_RGB2GRAY);
	Mat dstImageL, dstImageR;
	dstImageL.create(grayImageL.size(), grayImageL.type());
	dstImageR.create(grayImageR.size(), grayImageR.type());

	Mat drawingL, drawingR;
	openCvImageLtemp.copyTo(drawingL);
	openCvImageRtemp.copyTo(drawingR);

	//左图提取边缘
	GaussianBlur(grayImageL, grayImageL, Size(3, 3), 0, 0);
	//medianBlur(grayImageL, dstImageL, 9);
	adaptiveThreshold(grayImageL, image_binL, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 15, -1);
	Mat kernel = getStructuringElement(MORPH_RECT, Size(20, 1));
	morphologyEx(image_binL, image_binL, MORPH_OPEN, kernel);

	vector<Vec4i> Llines;
	HoughLinesP(image_binL, Llines, 1, CV_PI / 180, 110, 350, 15);
	vector<Point2f> LPoint1, LPoint2;   //提取两条线，将点分为两组
	Vec4i L1 = Llines[0];
	LPoint1.push_back(Point(L1[0], L1[1]));
	LPoint1.push_back(Point(L1[2], L1[3]));

	for (size_t i = 1; i < Llines.size(); i++)
	{
		Vec4i l = Llines[i];
		if ((l[1] > (L1[1] + L1[3]) / 2 - 50) && (l[1] < (L1[1] + L1[3]) / 2 + 50))
		{
			LPoint1.push_back(Point(l[0], l[1]));
			LPoint1.push_back(Point(l[2], l[3]));
		}
		else
		{
			LPoint2.push_back(Point(l[0], l[1]));
			LPoint2.push_back(Point(l[2], l[3]));
		}

		//line(drawingL, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
	}
	//左第一条线
	Vec4f Lline1;
	fitLine(LPoint1, Lline1, cv::DIST_L2, 0, 1e-2, 1e-2);
	//获取点斜式的点和斜率
	Point point0;
	point0.x = Lline1[2];
	point0.y = Lline1[3];
	double k = Lline1[1] / Lline1[0];

	//计算直线的端点(y = k(x - x0) + y0)
	Point point1, point2;
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 1626;
	point2.y = k * (1626 - point0.x) + point0.y;
	line(drawingL, point1, point2, Scalar(0, 0, 255), 1, CV_AA);

	vector<Point2f> LP1, RP1;   //左图第一条线上的特征点
	for (int i = 500; i < 1000; i = i + 50)
	{
		Point point;
		point.x = i;
		point.y = k * (i - point0.x) + point0.y;
		LP1.push_back(point);
	}
	//画左图特征点
	for (int i = 0; i < 10; i++)
	{
		circle(drawingL, LP1[i], 3, Scalar(0, 255, 0), 5);
	}

	//左第二条线
	Vec4f Lline2;
	fitLine(LPoint2, Lline2, cv::DIST_L2, 0, 1e-2, 1e-2);
	//获取点斜式的点和斜率
	point0.x = Lline2[2];
	point0.y = Lline2[3];
	k = Lline2[1] / Lline2[0];

	//计算直线的端点(y = k(x - x0) + y0)
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 1626;
	point2.y = k * (1626 - point0.x) + point0.y;
	line(drawingL, point1, point2, Scalar(0, 0, 255), 1, CV_AA);

	vector<Point2f> LP2, RP2;   //左图第二条线上的特征点
	for (int i = 700; i < 1100; i = i + 200)
	{
		Point point;
		point.x = i;
		point.y = k * (i - point0.x) + point0.y;
		LP2.push_back(point);
	}

	displayMat(ui.label, drawingL);

	//右图提取边缘
	GaussianBlur(grayImageR, grayImageR, Size(3, 3), 0, 0);
	adaptiveThreshold(grayImageR, image_binR, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 13, -1);
	morphologyEx(image_binR, image_binR, MORPH_OPEN, kernel);

	vector<Vec4i> Rlines;
	HoughLinesP(image_binR, Rlines, 1, CV_PI / 180, 110, 350, 15);
	vector<Point2f> RPoint1, RPoint2;   //提取两条线，将点分为两组
	Vec4i R1 = Rlines[0];
	RPoint1.push_back(Point(R1[0], R1[1]));
	RPoint1.push_back(Point(R1[2], R1[3]));
	for (size_t i = 0; i < Rlines.size(); i++)
	{
		Vec4i l = Rlines[i];
		if ((l[1] > (L1[1] + L1[3]) / 2 - 50) && (l[1] < (L1[1] + L1[3]) / 2 + 50))
		{
			RPoint1.push_back(Point(l[0], l[1]));
			RPoint1.push_back(Point(l[2], l[3]));
		}
		else
		{
			RPoint2.push_back(Point(l[0], l[1]));
			RPoint2.push_back(Point(l[2], l[3]));
		}
		//line(drawingR, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
	}
	//右第一条线
	Vec4f Rline1;
	fitLine(RPoint1, Rline1, cv::DIST_L2, 0, 1e-2, 1e-2);
	//获取点斜式的点和斜率
	point0.x = Rline1[2];
	point0.y = Rline1[3];
	k = Rline1[1] / Rline1[0];
	//计算直线的端点(y = k(x - x0) + y0)
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 1626;
	point2.y = k * (1626 - point0.x) + point0.y;
	line(drawingR, point1, point2, Scalar(0, 0, 255), 1, CV_AA);

	//计算右图第一条线与对应极线的交点作为右图对应特征点
	vector<Vec3f> epilines1;
	computeCorrespondEpilines(LP1, 1, F, epilines1);
	double linea[10], lineb[10], linec[10];
	for (int i = 0; i < 10; i++)
	{
		linea[i] = epilines1[i][0];
		lineb[i] = epilines1[i][1];
		linec[i] = epilines1[i][2];
	}
	for (int i = 0; i < 10; i++)
	{
		lineb[i] = lineb[i] / linea[i];
		linec[i] = linec[i] / linea[i];
		linea[i] = 1;
	}
	for (int i = 0; i < 10; i++)
	{
		calculateR(k, point0.x, point0.y, linea[i], lineb[i], linec[i], RP1);
	}
	//画右图极线和特征点
	for (int i = 0; i < 10; i++)
	{
		circle(drawingR, RP1[i], 3, Scalar(0, 255, 0), 5);
		//line(drawingR, Point(100, (-linea[i] * 100 - linec[i]) / lineb[i]),Point(1500, (-linea[i] * 1500 - linec[i]) / lineb[i]), Scalar(0, 0, 255), 2, CV_AA);
	}

	//计算特征点的三维坐标
	vector<Point3f>  WorldPoint1;
	calculateWorldPoint(WorldPoint1, LP1, RP1, M1, D1, M2, D2, R, T);

	//右第二条线
	Vec4f Rline2;
	fitLine(RPoint2, Rline2, cv::DIST_L2, 0, 1e-2, 1e-2);
	//获取点斜式的点和斜率
	point0.x = Rline2[2];
	point0.y = Rline2[3];
	k = Rline2[1] / Rline2[0];

	//计算直线的端点(y = k(x - x0) + y0)
	point1.x = 0;
	point1.y = k * (0 - point0.x) + point0.y;
	point2.x = 1626;
	point2.y = k * (1626 - point0.x) + point0.y;
	line(drawingR, point1, point2, Scalar(0, 0, 255), 1, CV_AA);

	//计算右图第二条线与对应极线的交点作为右图对应特征点
	vector<Vec3f> epilines2;
	computeCorrespondEpilines(LP2, 1, F, epilines2);
	double linea2[2], lineb2[2], linec2[2];
	for (int i = 0; i < 2; i++)
	{
		linea2[i] = epilines2[i][0];
		lineb2[i] = epilines2[i][1];
		linec2[i] = epilines2[i][2];
	}
	for (int i = 0; i < 2; i++)
	{
		lineb2[i] = lineb2[i] / linea2[i];
		linec2[i] = linec2[i] / linea2[i];
		linea2[i] = 1;
	}
	for (int i = 0; i < 2; i++)
	{
		calculateR(k, point0.x, point0.y, linea2[i], lineb2[i], linec2[i], RP2);
	}
	//计算右图第二条线的空间方程
	vector<Point3f>  WorldPoint2;
	calculateWorldPoint(WorldPoint2, LP2, RP2, M1, D1, M2, D2, R, T);

	displayMat(ui.label2, drawingR);

	double width = 0;
	calcwide(WorldPoint1, WorldPoint2, width);  //计算宽度
	stringstream p;
	p << "width:" << width << "mm" << endl;
	string s;
	s = p.str();
	QString information = QString::fromStdString(s);
	ui.resultshow->textCursor().insertText(information);
	ui.resultshow->textCursor().insertText("Measurement Completion!\n");


}

//找右图特征点
void QtGuiApplication::calculateR(double &k, double x0, double y0, double &A, double &B, double &C, vector<cv::Point2f> &Rpoints)
{
	if (k != -1 / B)
	{
		double x = (B*k*x0 - C - B * y0) / (1 + B * k);
		double y = (-k * x0 + y0 - k * C) / (1 + B * k);
		Rpoints.push_back(Point(x, y));
	}
	else
	{
		Rpoints.push_back(Point(0, 0));
	}
}
//计算宽度
void QtGuiApplication::calcwide(vector<cv::Point3f> &WorldPoint1, vector<cv::Point3f> &WorldPoint2, double &width)
{
	double x1 = WorldPoint2[0].x;
	double y1 = WorldPoint2[0].y;
	double z1 = WorldPoint2[0].z;
	double x2 = WorldPoint2[1].x;
	double y2 = WorldPoint2[1].y;
	double z2 = WorldPoint2[1].z;
	double a = x2 - x1;
	double b = y2 - y1;
	double c = z2 - z1;

	for (int i = 0;i < 10; i++)
	{
		double x3 = WorldPoint1[i].x;
		double y3 = WorldPoint1[i].y;
		double z3 = WorldPoint1[i].z;

		double e = x2 - x3;
		double f = y2 - y3;
		double g = z2 - z3;

		double fenzi = (b*g - f * c)*(b*g - f * c) + (c*e - a * g)*(c*e - a * g) + (a*f - e * b)*(a*f - e * b);
		double fenmu = a * a + b * b + c * c;
		width = width + sqrt(fenzi) / sqrt(fenmu);
	}
	width = width / 10;
}


/*----------------------------------------------------靶标测量----------------------------------------------------*/
void QtGuiApplication::droneclicked()
{
	ui.resultshow->clear();
	ui.resultshow->textCursor().insertText("Start Measurement!\n");
	//打开图像
	Mat oriImageL, oriImageR;
	openCvImageLtemp.copyTo(oriImageL);
	openCvImageRtemp.copyTo(oriImageR);
	//读取参数
	FileStorage fs("intrinsics.yml", FileStorage::READ);
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;
	fs["F"] >> F;
	fs.release();
	fs.open("extrinsics.yml", FileStorage::READ);
	fs["R"] >> R;
	fs["T"] >> T;

	Mat  image_binL, image_binR;
	Mat grayImageL, grayImageR;
	cvtColor(oriImageL, grayImageL, CV_BGR2GRAY);
	cvtColor(oriImageR, grayImageR, CV_BGR2GRAY);
	Mat dstImageL, dstImageR;
	dstImageL.create(grayImageL.size(), grayImageL.type());
	dstImageR.create(grayImageR.size(), grayImageR.type());

	//提取边缘
	medianBlur(grayImageL, dstImageL, 5);
	Canny(dstImageL, dstImageL, 130, 390, 3);
	threshold(dstImageL, image_binL, 100, 255, THRESH_BINARY);
	//imshow("erzhihuaL", image_binL);
	//waitKey(30);

	medianBlur(grayImageR, dstImageR, 5);
	Canny(dstImageR, dstImageR, 130, 390, 3);
	threshold(dstImageR, image_binR, 100, 255, THRESH_BINARY);
	//imshow("erzhihuaR", image_binR);
	//waitKey(30);


	//拟合左图像的圆
	int numL = 0;
	Mat srcImageL = Mat::zeros(oriImageL.size(), CV_8UC3);
	vector<Point2f>  LPoint;
	fit(image_binL, srcImageL, numL, LPoint);
	displayMat(ui.label, srcImageL);
	//imshow("ContoursL", srcImageL);
	//waitKey(30);
	QString informationL = informationL.fromLocal8Bit("左图共有%1个圆\n").arg(numL);
	ui.resultshow->textCursor().insertText(informationL);
	stringstream l;
	l << "LPoints:" << LPoint[0] << "," << LPoint[1] << "," << LPoint[2] << "," << LPoint[3] << endl;
	string lpoint = l.str();
	QString lpoints = QString::fromStdString(lpoint);
	ui.resultshow->textCursor().insertText(lpoints);

	//拟合右图像的圆
	int numR = 0;
	Mat srcImageR = Mat::zeros(oriImageR.size(), CV_8UC3);
	vector<Point2f>  RPoint;
	fit(image_binR, srcImageR, numR, RPoint);
	displayMat(ui.label2, srcImageR);
	//imshow("ContoursR", srcImageR);
	//waitKey(30);
	QString informationR = informationR.fromLocal8Bit("右图共有%1个圆\n").arg(numR);
	ui.resultshow->textCursor().insertText(informationR);
	stringstream Rp;
	Rp << "RPoints:" << RPoint[0] << "," << RPoint[1] << "," << RPoint[2] << "," << RPoint[3] << endl;
	string rpoint = Rp.str();
	QString rpoints = QString::fromStdString(rpoint);
	ui.resultshow->textCursor().insertText(rpoints);
	//世界坐标系
	vector<Point3f>  WorldPoint;
	calculateWorldPoint(WorldPoint, LPoint, RPoint, M1, D1, M2, D2, R, T);
	stringstream wp;
	wp << "World   Points:" << WorldPoint[0] << "," << WorldPoint[1] << "," << WorldPoint[2] << "," << WorldPoint[3] << endl;
	string wpoint = wp.str();
	QString wpoints = QString::fromStdString(wpoint);
	ui.resultshow->textCursor().insertText(wpoints);
	//求孔间距
	double length = 0, width = 0;
	calcrect(WorldPoint, length, width);
	double hole = (length + width) / 2 / (sqrt(numR) - 1);
	double diam = hole / 2;
	stringstream p;
	p << "Dot Center Spacing:" << hole << "mm," << "Dot Diameter:" << diam << "mm" << endl;
	string s;
	s = p.str();
	QString information = QString::fromStdString(s);
	ui.resultshow->textCursor().insertText(information);
	ui.resultshow->textCursor().insertText("Measurement Completion!\n");
}
//拟合椭圆
void QtGuiApplication::fit(Mat image_bin, Mat &srcImage, int &num, vector<cv::Point2f> &point)
{
	//检测圆
	Mat Image = Mat::zeros(image_bin.size(), CV_8UC1);
	vector<vector<cv::Point>> contourstemp;
	vector<Vec4i> hierarchy;
	vector<Point2f> centertemp;   //圆心点的集合

	cv::findContours(image_bin, contourstemp, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
	for (int i = 0; i < contourstemp.size(); i++)
	{
		Mat pointsf;
		if (contourstemp[i].size() < 5)
			continue;
		//将轮廓中的点转换为以Mat形式存储的2维点集(x,y)
		Mat(contourstemp[i]).convertTo(pointsf, CV_32F);
		RotatedRect box = fitEllipse(pointsf);
		double S1 = contourArea(contourstemp[i]);
		double S2 = box.size.height*box.size.width / 4 * CV_PI;
		if ((S1 / S2 < 0.98) || (S1 / S2 > 1.02))
			continue;
		drawContours(Image, contourstemp, i, Scalar(255), 1, 8, hierarchy);
	}
	vector<vector<cv::Point>> contours;
	cv::findContours(Image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++)
	{
		Mat pointsf;
		//将轮廓中的点转换为以Mat形式存储的2维点集(x,y)
		Mat(contours[i]).convertTo(pointsf, CV_32F);
		RotatedRect box = fitEllipse(pointsf);
		//绘制椭圆
		ellipse(srcImage, box, Scalar(255, 255, 255), 1, CV_AA);
		num++;
		//圆心
		Point2f center;
		center = box.center;
		centertemp.push_back(center);
	}
	double tempx = 0, tempy = 0;
	for (int i = 0; i < centertemp.size(); i++)
	{
		tempx = tempx + centertemp[i].x;
		tempy = tempy + centertemp[i].y;
	}
	tempx = tempx / num;
	tempy = tempy / num;
	Point2f point1, point2, point3, point4;
	point1.x = point2.x = point3.x = point4.x = tempx;
	point1.y = point2.y = point3.y = point4.y = tempy;
	for (int i = 0; i < centertemp.size(); i++)
	{
		if ((centertemp[i].x < tempx) && (centertemp[i].y < tempy))  //左上角
		{
			double f1 = (centertemp[i].x - tempx)*(centertemp[i].x - tempx) + (centertemp[i].y - tempy)*(centertemp[i].y - tempy);
			double f2 = (point1.x - tempx)*(point1.x - tempx) + (point1.y - tempy)*(point1.y - tempy);
			if (f1 > f2)
			{
				point1.x = centertemp[i].x;
				point1.y = centertemp[i].y;
			}
		}
		else if ((centertemp[i].x > tempx) && (centertemp[i].y < tempy))  //右上角
		{
			double f1 = (centertemp[i].x - tempx)*(centertemp[i].x - tempx) + (centertemp[i].y - tempy)*(centertemp[i].y - tempy);
			double f2 = (point2.x - tempx)*(point2.x - tempx) + (point2.y - tempy)*(point2.y - tempy);
			if (f1 > f2)
			{
				point2.x = centertemp[i].x;
				point2.y = centertemp[i].y;
			}
		}
		else if ((centertemp[i].x > tempx) && (centertemp[i].y > tempy))  //右下角
		{
			double f1 = (centertemp[i].x - tempx)*(centertemp[i].x - tempx) + (centertemp[i].y - tempy)*(centertemp[i].y - tempy);
			double f2 = (point3.x - tempx)*(point3.x - tempx) + (point3.y - tempy)*(point3.y - tempy);
			if (f1 > f2)
			{
				point3.x = centertemp[i].x;
				point3.y = centertemp[i].y;
			}
		}
		else  //左下角
		{
			double f1 = (centertemp[i].x - tempx)*(centertemp[i].x - tempx) + (centertemp[i].y - tempy)*(centertemp[i].y - tempy);
			double f2 = (point4.x - tempx)*(point4.x - tempx) + (point4.y - tempy)*(point4.y - tempy);
			if (f1 > f2)
			{
				point4.x = centertemp[i].x;
				point4.y = centertemp[i].y;
			}
		}
	}
	point.push_back(point1);
	point.push_back(point2);
	point.push_back(point3);
	point.push_back(point4);
}
//计算矩形的长宽
void QtGuiApplication::calcrect(vector<cv::Point3f> &calcWorldPoint, double &length, double &width)
{
	length = sqrt((calcWorldPoint[0].x - calcWorldPoint[1].x)*(calcWorldPoint[0].x - calcWorldPoint[1].x) +
		(calcWorldPoint[0].y - calcWorldPoint[1].y)*(calcWorldPoint[0].y - calcWorldPoint[1].y) +
		(calcWorldPoint[0].z - calcWorldPoint[1].z)*(calcWorldPoint[0].z - calcWorldPoint[1].z));
	length = length + sqrt((calcWorldPoint[3].x - calcWorldPoint[2].x)*(calcWorldPoint[3].x - calcWorldPoint[2].x) +
		(calcWorldPoint[3].y - calcWorldPoint[2].y)*(calcWorldPoint[3].y - calcWorldPoint[2].y) +
		(calcWorldPoint[3].z - calcWorldPoint[2].z)*(calcWorldPoint[3].z - calcWorldPoint[2].z));
	length = length / 2;

	width = sqrt((calcWorldPoint[1].x - calcWorldPoint[2].x)*(calcWorldPoint[1].x - calcWorldPoint[2].x) +
		(calcWorldPoint[1].y - calcWorldPoint[2].y)*(calcWorldPoint[1].y - calcWorldPoint[2].y) +
		(calcWorldPoint[1].z - calcWorldPoint[2].z)*(calcWorldPoint[1].z - calcWorldPoint[2].z));
	width = width + sqrt((calcWorldPoint[3].x - calcWorldPoint[0].x)*(calcWorldPoint[3].x - calcWorldPoint[0].x) +
		(calcWorldPoint[3].y - calcWorldPoint[0].y)*(calcWorldPoint[3].y - calcWorldPoint[0].y) +
		(calcWorldPoint[3].z - calcWorldPoint[0].z)*(calcWorldPoint[3].z - calcWorldPoint[0].z));
	width = width / 2;
}
/*----------------------------------------------------图像显示----------------------------------------------------*/
void QtGuiApplication::showimage()        //左图像显示
{
	//Pylon自动初始化和终止
	Pylon::PylonAutoInitTerm autoInitTerm;
	ui.resultshow->clear();

	//创建相机对象（左摄像头）
	CTlFactory& TlFactory = CTlFactory::GetInstance();
	DeviceInfoList_t Devices;
	if (TlFactory.EnumerateDevices(Devices) < 1)
	{
		ui.open_camera->setEnabled(true);
		ui.close_camera->setEnabled(false);
		ui.save_camera->setEnabled(false);
		ui.open_left->setEnabled(true);
		ui.open_right->setEnabled(true);
		ui.resultshow->textCursor().insertText("Left camera open failed!\n");
		timer->stop();
		return;
	}
	CInstantCamera  camera;
	camera.Attach(TlFactory.CreateDevice(Devices[1]));


	GenApi::INodeMap& nodemap = camera.GetNodeMap();

	//打开相机
	camera.Open();

	//获取相机成像宽度和高度
	GenApi::CIntegerPtr width = nodemap.GetNode("Width");
	GenApi::CIntegerPtr height = nodemap.GetNode("Height");


	//设置相机最大缓冲区,默认为10
	camera.MaxNumBuffer = 5;

	// 新建pylon ImageFormatConverter对象.
	CImageFormatConverter formatConverter;

	//确定输出像素格式
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;

	// 创建一个Pylonlmage后续将用来创建OpenCV images
	CPylonImage pylonImage;


	//声明一个整形变量用来计数抓取的图像，以及创建文件名索引
	int grabbedlmages = 0;

	// 新建一个OpenCV video creator对象.
	VideoWriter cvVideoCreator;

	// 视频文件名
	std::string videoFileName = "openCvVideo.avi";

	// 定义视频帧大小
	cv::Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());

	//设置视频编码类型和帧率，有三种选择
	// 帧率必须小于等于相机成像帧率
	cvVideoCreator.open(videoFileName, CV_FOURCC('D', 'I', 'V', 'X'), 10, frameSize, true);
	//cvVideoCreator.open(videoFileName, CV_F0URCC('M','P',,4','2’), 20, frameSize, true);
	//cvVideoCreator.open(videoFileName, CV_FOURCC('M', '3', 'P', 'G'), 20, frameSize, true);


	// 开始抓取c_countOfImagesToGrab images.
	//相机默认设置连续抓取模式
	camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);


	//抓取结果数据指针
	CGrabResultPtr ptrGrabResult;

	// 当c_countOfImagesToGrab images获取恢复成功时，Camera.StopGrabbing() 
	//被RetrieveResult()方法自动调用停止抓取
	waitKey(0);
	while (camera.IsGrabbing())

	{
		// 等待接收和恢复图像，超时时间设置为1000 ms.
		camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);

		//如果图像抓取成功
		if (ptrGrabResult->GrabSucceeded())
		{

			//将抓取的缓冲数据转化成pylon image.
			formatConverter.Convert(pylonImage, ptrGrabResult);

			// 将 pylon image转成OpenCV image.
			openCvImageL = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
			displayMat(ui.label, openCvImageL);  //显示
			QString information = information.fromLocal8Bit("%1*%2").arg(openCvImageL.cols).arg(openCvImageL.rows);
			ui.lsize->setText(information);
			if (saveflagL == 1)
			{
				saveL(openCvImageL);

			}
		}
	}
	saveflagL = 0;
}

void QtGuiApplication::showimage2()        //右图像显示
{
	//Pylon自动初始化和终止
	Pylon::PylonAutoInitTerm autoInitTerm;

	//创建相机对象（左摄像头）
	CTlFactory& TlFactory = CTlFactory::GetInstance();
	DeviceInfoList_t Devices;
	if (TlFactory.EnumerateDevices(Devices) < 1)
	{
		ui.open_camera->setEnabled(true);
		ui.close_camera->setEnabled(false);
		ui.save_camera->setEnabled(false);
		ui.open_left->setEnabled(true);
		ui.open_right->setEnabled(true);
		ui.resultshow->textCursor().insertText("Right camera open failed!\n");
		timer->stop();
		return;
	}
	CInstantCamera  camera;
	camera.Attach(TlFactory.CreateDevice(Devices[0]));


	GenApi::INodeMap& nodemap = camera.GetNodeMap();

	//打开相机
	camera.Open();

	//获取相机成像宽度和高度
	GenApi::CIntegerPtr width = nodemap.GetNode("Width");
	GenApi::CIntegerPtr height = nodemap.GetNode("Height");


	//设置相机最大缓冲区,默认为10
	camera.MaxNumBuffer = 5;

	// 新建pylon ImageFormatConverter对象.
	CImageFormatConverter formatConverter;

	//确定输出像素格式
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;

	// 创建一个Pylonlmage后续将用来创建OpenCV images
	CPylonImage pylonImage;


	//声明一个整形变量用来计数抓取的图像，以及创建文件名索引
	int grabbedlmages = 0;

	// 新建一个OpenCV video creator对象.
	VideoWriter cvVideoCreator;

	// 视频文件名
	std::string videoFileName = "openCvVideo.avi";

	// 定义视频帧大小
	cv::Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());

	//设置视频编码类型和帧率，有三种选择
	// 帧率必须小于等于相机成像帧率
	cvVideoCreator.open(videoFileName, CV_FOURCC('D', 'I', 'V', 'X'), 10, frameSize, true);
	//cvVideoCreator.open(videoFileName, CV_F0URCC('M','P',,4','2’), 20, frameSize, true);
	//cvVideoCreator.open(videoFileName, CV_FOURCC('M', '3', 'P', 'G'), 20, frameSize, true);


	// 开始抓取c_countOfImagesToGrab images.
	//相机默认设置连续抓取模式
	camera.StartGrabbing(c_countOfImagesToGrab, GrabStrategy_LatestImageOnly);


	//抓取结果数据指针
	CGrabResultPtr ptrGrabResult;

	// 当c_countOfImagesToGrab images获取恢复成功时，Camera.StopGrabbing() 
	//被RetrieveResult()方法自动调用停止抓取
	waitKey(0);
	while (camera.IsGrabbing())

	{
		// 等待接收和恢复图像，超时时间设置为1000 ms.
		camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);

		//如果图像抓取成功
		if (ptrGrabResult->GrabSucceeded())
		{

			//将抓取的缓冲数据转化成pylon image.
			formatConverter.Convert(pylonImage, ptrGrabResult);

			// 将 pylon image转成OpenCV image.
			openCvImageR = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
			displayMat(ui.label2, openCvImageR);  //显示
			QString information = information.fromLocal8Bit("%1*%2").arg(openCvImageR.cols).arg(openCvImageR.rows);
			ui.rsize->setText(information);
			if (saveflagR == 1)
			{
				saveR(openCvImageR);
			}
		}
	}
	saveflagR = 0;
}

void QtGuiApplication::displayMat(QLabel *label, Mat image)     //图像转换
{

	Mat rgb;
	QImage img;
	cv::Size cvS;
	QSize qtS;
	qtS = label->size();
	cvS.height = label->height();
	cvS.width = label->width();
	cv::resize(image, image, cvS);//图像按照label的大小进行显示
	if (image.channels() == 3)
	{
		cvtColor(image, rgb, CV_BGR2RGB);
		img = QImage((const unsigned char*)(rgb.data),
			rgb.cols, rgb.rows,
			rgb.cols*rgb.channels(),
			QImage::Format_RGB888);
	}
	else
	{
		img = QImage((const unsigned char*)(image.data),
			image.cols, image.rows,
			image.cols*image.channels(),
			QImage::Format_Indexed8);
	}
	label->setPixmap(QPixmap::fromImage(img));//显示
}
//保存左图
void QtGuiApplication::saveL(Mat image)
{
	QString filename1 = QFileDialog::getSaveFileName(this, tr("save left_image"), "./left**.jpg", tr("Images (*.jpg)")); //选择路径    
	if (filename1.isEmpty())
		return;
	else
	{
		string s = filename1.toStdString();
		imwrite(s, image);
	}
}
//保存右图
void QtGuiApplication::saveR(Mat image)
{
	QString filename1 = QFileDialog::getSaveFileName(this, tr("save right_image"), "./right**.jpg", tr("Images (*.jpg)")); //选择路径    
	if (filename1.isEmpty())
		return;
	else
	{
		string s = filename1.toStdString();
		imwrite(s, image);
	}
}
void QtGuiApplication::change()
{
	ui.resultshow->moveCursor(QTextCursor::End);
}

//计算世界坐标
void QtGuiApplication::calculateWorldPoint(vector<cv::Point3f> &calcWorldPoint, vector<cv::Point2f> &Lpoints, vector<cv::Point2f> &Rpoints,
	Mat &l_cameraMatrix, Mat &l_disCoeff, Mat &r_cameraMatrix, Mat &r_disCoeff, Mat &R, Mat &T)
{
	//undistort the image point
	cv::undistortPoints(Lpoints, Lpoints, l_cameraMatrix, l_disCoeff);
	cv::undistortPoints(Rpoints, Rpoints, r_cameraMatrix, r_disCoeff);
	//pixel to mm
	for (int i = 0; i < Lpoints.size(); i++)
	{
		Lpoints[i].x = Lpoints[i].x*l_cameraMatrix.at<double>(0, 0) + l_cameraMatrix.at<double>(0, 2);
		Lpoints[i].y = Lpoints[i].y*l_cameraMatrix.at<double>(1, 1) + l_cameraMatrix.at<double>(1, 2);
	}

	for (int i = 0; i < Rpoints.size(); i++)
	{
		Rpoints[i].x = Rpoints[i].x*r_cameraMatrix.at<double>(0, 0) + r_cameraMatrix.at<double>(0, 2);
		Rpoints[i].y = Rpoints[i].y*r_cameraMatrix.at<double>(1, 1) + r_cameraMatrix.at<double>(1, 2);
	}

	Mat P1 = (cv::Mat_<double>(3, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	Mat P2 = (cv::Mat_<double>(3, 4) << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	for (int i = 0; i < l_cameraMatrix.rows; i++)
	{
		for (int j = 0; j < l_cameraMatrix.cols; j++)
		{
			//cout << P1.at<double>(i, j) << endl << left_camera_.cameraMatrix.at<double>(i, j) << endl;
			P1.at<double>(i, j) = l_cameraMatrix.at<double>(i, j);
			P2.at<double>(i, j) = R.at<double>(i, j);
		}
	}
	for (int k = 0; k < 3; k++)
	{
		P1.at<double>(k, 3) = 0;
		P2.at<double>(k, 3) = T.at<double>(k, 0);
	}
	P2 = r_cameraMatrix * P2;   //相机矩阵

	cv::Mat world_mat;
	cv::triangulatePoints(P1, P2, Lpoints, Rpoints, world_mat);
	//4. convert mat to vector
	//vector<cv::Point3f> world_points;
	for (int k = 0; k < world_mat.cols; k++)
	{
		cv::Point3f world3_ii;
		world3_ii.x = world_mat.at<float>(0, k) / world_mat.at<float>(3, k);
		world3_ii.y = world_mat.at<float>(1, k) / world_mat.at<float>(3, k);
		world3_ii.z = world_mat.at<float>(2, k) / world_mat.at<float>(3, k);
		calcWorldPoint.push_back(world3_ii);
	}
}