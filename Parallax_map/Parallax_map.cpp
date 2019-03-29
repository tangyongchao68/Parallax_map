// Parallax_map.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/opencv.hpp>
#include <highgui.hpp>
#include<calib3d\calib3d.hpp>
#include "cv.h"
#include <cv.hpp>
#include <iostream>


#include "opencv2/imgproc/imgproc.hpp"    
#include "opencv2/highgui/highgui.hpp"  


using namespace std;
using namespace cv;

/* SAD(Sum of absolute differences)是一种图像匹配算法。
基本思想：差的绝对值之和。此算法常用于图像块匹配，将每个像素对应数值之差的绝对值求和，
据此评估两个图像块的相似度。该算法快速、但并不精确，通常用于多级处理的初步筛选。*/
class SAD {
private:
	int winSize;//卷积核尺寸
	int DSR;//视差搜索范围
public:
	SAD() :winSize(7), DSR(30) {}
	SAD(int _winSize, int _DSR) :winSize(_winSize), DSR(_DSR) {}
	Mat computerSAD(Mat&L, Mat&R);//计算SAD
};
Mat SAD::computerSAD(Mat&L, Mat&R) {
	int Height = L.rows;
	int Width = L.cols;
	Mat Kernel_L(Size(winSize, winSize), CV_8U, Scalar::all(0));
	//CV_8U:0~255的值，大多数图像/视频的格式，该段设置全0矩阵
	Mat Kernel_R(Size(winSize, winSize), CV_8U, Scalar::all(0));
	Mat Disparity(Height, Width, CV_8U, Scalar(0));


	for (int i = 0; i < Width - winSize; ++i) {
		for (int j = 0; j < Height - winSize; ++j) {
			Kernel_L = L(Rect(i, j, winSize, winSize));//L为做图像，Kernel为这个范围内的左图
			Mat MM(1, DSR, CV_32F, Scalar(0));//定义匹配范围

			for (int k = 0; k < DSR; ++k) {
				int x = i - k;
				if (x >= 0) {
					Kernel_R = R(Rect(x, j, winSize, winSize));
					Mat Dif;
					absdiff(Kernel_L, Kernel_R, Dif);
					Scalar ADD = sum(Dif);
					float a = ADD[0];
					MM.at<float>(k) = a;
				}
				Point minLoc;
				minMaxLoc(MM, NULL, NULL, &minLoc, NULL);

				int loc = minLoc.x;
				Disparity.at<char>(j, i) = loc * 16;
			}
			double rate = double(i) / (Width);
			cout << "已完成" << setprecision(2) << rate * 100 << "%" << endl;
		}
	}
	return Disparity;
}

/*BM算法：速度很快，效果一般*/
void BM()
{
//	IplImage * img1 = cvLoadImage("./rectifyImageL.bmp", 0);
//	IplImage * img2 = cvLoadImage("./rectifyImageR.bmp", 0);
	IplImage * img1 = cvLoadImage("./im0.png", 0);
	IplImage * img2 = cvLoadImage("./im1.png", 0);



	namedWindow("leftimag");
	namedWindow("rightimag");
	cvShowImage("leftimag", img1);
	cvShowImage("rightimag", img2);



	CvStereoBMState* BMState = cvCreateStereoBMState();
	assert(BMState);
	BMState->preFilterSize = 9;
	BMState->preFilterCap = 31;
	BMState->SADWindowSize = 15;
	BMState->minDisparity = 0;
	BMState->numberOfDisparities = 64;
	BMState->textureThreshold = 10;
	BMState->uniquenessRatio = 15;
	BMState->speckleWindowSize = 100;
	BMState->speckleRange = 32;
	BMState->disp12MaxDiff = 1;

	CvMat* disp = cvCreateMat(img1->height, img1->width, CV_16S);
	CvMat* vdisp = cvCreateMat(img1->height, img1->width, CV_8U);
	int64 t = getTickCount();
	cvFindStereoCorrespondenceBM(img1, img2, disp, BMState);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	cvSave("disp.xml", disp);
	cvNormalize(disp, vdisp, 0, 255, CV_MINMAX);
	cvNamedWindow("BM_disparity", 0);
	cvShowImage("BM_disparity", vdisp);
	cvWaitKey(0);
	//cvSaveImage("cones\\BM_disparity.png",vdisp);
	cvReleaseMat(&disp);
	cvReleaseMat(&vdisp);
	cvDestroyWindow("leftimag");
	cvDestroyWindow("rightimag");
	cvDestroyWindow("BM_disparity");
}
/*GC算法 效果最好，速度最慢
void GC()
{
	IplImage * img1 = cvLoadImage("left.png", 0);
	IplImage * img2 = cvLoadImage("right.png", 0);
	CvStereoGCState* GCState = cvCreateStereoGCState(64, 3);
	assert(GCState);
	cout << "start matching using GC" << endl;
	CvMat* gcdispleft = cvCreateMat(img1->height, img1->width, CV_16S);
	CvMat* gcdispright = cvCreateMat(img2->height, img2->width, CV_16S);
	CvMat* gcvdisp = cvCreateMat(img1->height, img1->width, CV_8U);
	int64 t = getTickCount();
	cvFindStereoCorrespondenceGC(img1, img2, gcdispleft, gcdispright, GCState);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	//cvNormalize(gcdispleft,gcvdisp,0,255,CV_MINMAX);
	//cvSaveImage("GC_left_disparity.png",gcvdisp);
	cvNormalize(gcdispright, gcvdisp, 0, 255, CV_MINMAX);
	cvSaveImage("GC_right_disparity.png", gcvdisp);


	cvNamedWindow("GC_disparity", 0);
	cvShowImage("GC_disparity", gcvdisp);
	cvWaitKey(0);
	cvReleaseMat(&gcdispleft);
	cvReleaseMat(&gcdispright);
	cvReleaseMat(&gcvdisp);
}
*/
/*SGBM算法

作为一种全局匹配算法，立体匹配的效果明显好于局部匹配算法，但是同时复杂度上也要远远大于局部匹配算法。

void SGBM()
{

	IplImage * img1 = cvLoadImage("left.png", 0);
	IplImage * img2 = cvLoadImage("right.png", 0);
	cv::StereoSGBM sgbm;
	int SADWindowSize = 9;
	sgbm.preFilterCap = 63;
	sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;
	int cn = img1->nChannels;
	int numberOfDisparities = 64;
	sgbm.P1 = 8 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.P2 = 32 * cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
	sgbm.minDisparity = 0;
	sgbm.numberOfDisparities = numberOfDisparities;
	sgbm.uniquenessRatio = 10;
	sgbm.speckleWindowSize = 100;
	sgbm.speckleRange = 32;
	sgbm.disp12MaxDiff = 1;
	Mat disp, disp8;
	int64 t = getTickCount();
	sgbm((Mat)img1, (Mat)img2, disp);
	t = getTickCount() - t;
	cout << "Time elapsed:" << t * 1000 / getTickFrequency() << endl;
	disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

	namedWindow("left", 1);
	cvShowImage("left", img1);
	namedWindow("right", 1);
	cvShowImage("right", img2);
	namedWindow("disparity", 1);
	imshow("disparity", disp8);
	waitKey();
	imwrite("sgbm_disparity.png", disp8);
	cvDestroyAllWindows();
}
*/

int main()
{
//	Mat left = imread("./im0.png");
//	Mat right = imread("./im1.png");
	BM();
	Mat left = imread("./rectifyImageL.bmp");
	Mat right = imread("./rectifyImageR.bmp");
	//-------图像显示-----------
	namedWindow("leftimag");
	imshow("leftimag", left);

	namedWindow("rightimag");
	imshow("rightimag", right);
	//--------由SAD求取视差图-----
	Mat Disparity;

	SAD mySAD(7, 30);
	Disparity = mySAD.computerSAD(left, right);
	//-------结果显示------
	namedWindow("Disparity");
	imshow("Disparity", Disparity);
	//-------收尾------
	waitKey(0);
	return 0;

}

