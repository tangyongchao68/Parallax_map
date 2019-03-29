// Parallax_map.cpp : �������̨Ӧ�ó������ڵ㡣
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

/* SAD(Sum of absolute differences)��һ��ͼ��ƥ���㷨��
����˼�룺��ľ���ֵ֮�͡����㷨������ͼ���ƥ�䣬��ÿ�����ض�Ӧ��ֵ֮��ľ���ֵ��ͣ�
�ݴ���������ͼ�������ƶȡ����㷨���١���������ȷ��ͨ�����ڶ༶����ĳ���ɸѡ��*/
class SAD {
private:
	int winSize;//����˳ߴ�
	int DSR;//�Ӳ�������Χ
public:
	SAD() :winSize(7), DSR(30) {}
	SAD(int _winSize, int _DSR) :winSize(_winSize), DSR(_DSR) {}
	Mat computerSAD(Mat&L, Mat&R);//����SAD
};
Mat SAD::computerSAD(Mat&L, Mat&R) {
	int Height = L.rows;
	int Width = L.cols;
	Mat Kernel_L(Size(winSize, winSize), CV_8U, Scalar::all(0));
	//CV_8U:0~255��ֵ�������ͼ��/��Ƶ�ĸ�ʽ���ö�����ȫ0����
	Mat Kernel_R(Size(winSize, winSize), CV_8U, Scalar::all(0));
	Mat Disparity(Height, Width, CV_8U, Scalar(0));


	for (int i = 0; i < Width - winSize; ++i) {
		for (int j = 0; j < Height - winSize; ++j) {
			Kernel_L = L(Rect(i, j, winSize, winSize));//LΪ��ͼ��KernelΪ�����Χ�ڵ���ͼ
			Mat MM(1, DSR, CV_32F, Scalar(0));//����ƥ�䷶Χ

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
			cout << "�����" << setprecision(2) << rate * 100 << "%" << endl;
		}
	}
	return Disparity;
}

/*BM�㷨���ٶȺܿ죬Ч��һ��*/
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
/*GC�㷨 Ч����ã��ٶ�����
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
/*SGBM�㷨

��Ϊһ��ȫ��ƥ���㷨������ƥ���Ч�����Ժ��ھֲ�ƥ���㷨������ͬʱ���Ӷ���ҲҪԶԶ���ھֲ�ƥ���㷨��

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
	//-------ͼ����ʾ-----------
	namedWindow("leftimag");
	imshow("leftimag", left);

	namedWindow("rightimag");
	imshow("rightimag", right);
	//--------��SAD��ȡ�Ӳ�ͼ-----
	Mat Disparity;

	SAD mySAD(7, 30);
	Disparity = mySAD.computerSAD(left, right);
	//-------�����ʾ------
	namedWindow("Disparity");
	imshow("Disparity", Disparity);
	//-------��β------
	waitKey(0);
	return 0;

}

