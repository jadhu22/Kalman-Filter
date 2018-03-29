///////////////////////////////////////////////////
//////// Author: Akash Jadhav ////////////////////
//////////////////////////////////////////////////

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include<Windows.h>
#include<conio.h>           
#include<stdlib.h>
using namespace std;
using namespace cv;



int main()
{

	HANDLE hSerial = CreateFile("COM9", GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

	if (hSerial != INVALID_HANDLE_VALUE)
	{
		cout << "Port opened!" << endl;

		DCB dcbSerialParams;
		GetCommState(hSerial, &dcbSerialParams);

		dcbSerialParams.BaudRate = CBR_9600;
		dcbSerialParams.ByteSize = 32;
		dcbSerialParams.Parity = NOPARITY;
		//dcbSerialParams.StopBits = ONESTOPBIT;

		SetCommState(hSerial, &dcbSerialParams);
	}
	else
	{
		if (GetLastError() == ERROR_FILE_NOT_FOUND)
		{
			cout << "Serial port doesn't exist!" << endl;
		}

		cout << "Error" << endl;
	}
	BYTE Byte;

	
	char outputChars[] = "C";
	char inputChars[] = "c";
	char e[50];
	DWORD btsIO;
	DWORD dwBytesTransferred;

	namedWindow("controlHSV", CV_WINDOW_AUTOSIZE);

	int iLowH = 0;
	int iHighH = 0;
	int iLowS = 0;
	int iHighS = 0;
	int iLowV = 0;
	int iHighV = 0;

	createTrackbar("LowH", "controlHSV", &iLowH, 179);
	createTrackbar("HighH", "controlHSV", &iHighH, 179);
	createTrackbar("LowS", "controlHSV", &iLowS, 255);
	createTrackbar("HighS", "controlHSV", &iHighS, 255);
	createTrackbar("LowV", "controlHSV", &iLowV, 255);
	createTrackbar("HighV", "controlHSV", &iHighV, 255);
    
	Mat frame;

	int stateSize = 6;
	int measSize = 4;
	int contrSize = 0;

	unsigned int type = CV_32F;
	KalmanFilter kf(stateSize, measSize, contrSize, type);

	Mat state(stateSize, 1, type);
	Mat meas(measSize, 1, type);
	

	setIdentity(kf.transitionMatrix);

	kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
	kf.measurementMatrix.at<float>(0) = 1.0f;
	kf.measurementMatrix.at<float>(7) = 1.0f;
	kf.measurementMatrix.at<float>(16) = 1.0f;
	kf.measurementMatrix.at<float>(23) = 1.0f;

	//setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
	kf.processNoiseCov.at<float>(0) = 1e-1;
	kf.processNoiseCov.at<float>(7) = 1e-1;
	kf.processNoiseCov.at<float>(14) = 2.0f;
	//kf.processNoiseCov.at<float>(14) = 0;
	kf.processNoiseCov.at<float>(21) = 2.0f;
	
	//kf.processNoiseCov.at<float>(21) = 0;
	kf.processNoiseCov.at<float>(28) = 1e-1;
	kf.processNoiseCov.at<float>(35) = 1e-1;


	setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
	
	VideoCapture cap(0);


	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1100);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	char ch = 0;

	double ticks = 0;
	bool found = false;

	int nf = 0;

	while (ch != 'q')
	{
		double precTick = ticks;
		ticks = (double) getTickCount();

		double dT = (ticks - precTick) / getTickFrequency();
		

		cap >> frame;

		Mat res;
		frame.copyTo(res);

		if (found)
		{
			
			kf.transitionMatrix.at<float>(2) = dT;
			kf.transitionMatrix.at<float>(9) = dT;


			cout << "dT:" << endl << dT << endl;

			state = kf.predict();
			cout << "State post:" << endl << state << endl;

			Rect predRect;
			predRect.width = state.at<float>(4);
			predRect.height = state.at<float>(5);
			predRect.x = state.at<float>(0) - predRect.width / 2;
			predRect.y = state.at<float>(1) - predRect.height / 2;
			
			Point center;
			center.x = state.at<float>(0);
			center.y = state.at<float>(1);
			circle(res, center, 2, CV_RGB(255,0,0), -1);

			rectangle(res, predRect, CV_RGB(255,0,0), 2);

			
		}


		Mat blur;
		GaussianBlur(frame, blur, cv::Size(5, 5), 3.0, 3.0);
	

		Mat frmHsv;
		cvtColor(blur, frmHsv, CV_BGR2HSV);
		

		Mat rangeRes = Mat::zeros(frame.size(), CV_8UC1);
		
		//inRange(frmHsv, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), rangeRes);
		inRange(frmHsv, Scalar(21, 91, 220), Scalar(92, 255, 255), rangeRes);

		erode(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 1);
		dilate(rangeRes, rangeRes, cv::Mat(), cv::Point(-1, -1), 1);
		

		imshow("Threshold", rangeRes);
		

		vector<vector<cv::Point> > contours;
		findContours(rangeRes, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
		

		vector<vector<cv::Point> > balls;
		vector<cv::Rect> ballsBox;
		
		
		
		for (size_t i = 0; i < contours.size(); i++)
		{
			Rect bBox;
			bBox = boundingRect(contours[i]);

			float ratio = (float)bBox.width / (float)bBox.height;
			if (ratio > 1.0f)
				ratio = 1.0f / ratio;

			if (ratio > 0.1 && bBox.area() >= 500)
			{
				balls.push_back(contours[i]);
				ballsBox.push_back(bBox);
			}
		}

		
	

		for (size_t i = 0; i < balls.size(); i++)
		{
			
			rectangle(res, ballsBox[i], CV_RGB(0, 255, 0), 2);

			Point center;
			
			center.x = ballsBox[i].x + ballsBox[i].width / 2;
			center.y = ballsBox[i].y + ballsBox[i].height / 2;
			circle(res, center, 2, CV_RGB(0, 255, 0), -1);
			//cout << "X = " << center.x << " & " << "Y = " << center.y << endl;
			
			
		}
		

		if (balls.size() == 0)
		{
			  nf++;
			   
			if (nf >= 1000)
			{
				found = false;
			}

		}
		else
		{
			  nf = 0;

			meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
			meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
			meas.at<float>(2) = (float)ballsBox[0].width;
			meas.at<float>(3) = (float)ballsBox[0].height;
			
			if (!found)
			{

				kf.errorCovPre.at<float>(0) = 1;
				kf.errorCovPre.at<float>(7) = 1;
				kf.errorCovPre.at<float>(14) = 1;
				kf.errorCovPre.at<float>(21) = 1;
				kf.errorCovPre.at<float>(28) = 1;
				kf.errorCovPre.at<float>(35) = 1;

				state.at<float>(0) = meas.at<float>(0);
				state.at<float>(1) = meas.at<float>(1);
				state.at<float>(2) = 0;
				state.at<float>(3) = 0;
				state.at<float>(4) = meas.at<float>(2);
				state.at<float>(5) = meas.at<float>(3);
				
				found = true;
			}
			else
			kf.correct(meas);
			
	
		}

		imshow("Tracking", res);
		
		ch = waitKey(1);
	}


	return 0;
}
