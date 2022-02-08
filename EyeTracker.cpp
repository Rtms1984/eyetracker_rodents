
#include <iostream>
#include <sstream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>    // std::sort std::min_element
#include <vector>
#include <iterator>  // std::begin, std::end
#include <windows.h>
#include <direct.h>
#include <numeric>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#define WIN64
#define PI 3.1428571

#include "PuRe.h"
#include "GUIUtils.h"
#include "StereoCams.h"
#include "Calibration.h"
#include <pylon/PylonIncludes.h>



#ifndef _CRT_SECURE_NO_WARNINGS
# define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

extern int alpha_slider;
extern int alpha_slider_max;
extern int sThreshold;
extern int BORDER;  
extern Rect m_BtnNormal;
extern Rect m_BtnFast;
extern int FastPlot;


// #define RISELEZIONA   // per  attivare la selezione delle immagini della checkboard da telecamera stereo (s per  snapshot, q per quit)
#define FROMCAMERA
//#define FROMAVIFILE  // non implementato
//#define  SAVEONTRIGGER


typedef struct mystructtag
{
	float x1;
	float x2;
	bool valid = false;
} xcoords;


typedef struct mystructtag1
{
	float userMinPupilDiameterPx;
	float userMaxPupilDiameterPx;

	float VerticalCenterTh;
	float TotalHeightDiffTh;
	int YRange;
	int YStep;
	bool ShowEdges;
	// search for edges of the pupil
	int half_x_search;
	int half_x_peak;
	float nonEdgePixRatio; // 19.01.2022
} params;

typedef struct mystructtag2
{
	Point2f P_L;
	Point2f P_R;
} couple;

typedef struct mystructtag4
{
	double distTh = 0.02; // diatance from the plane to be considered inliner.
	double probGood = 0.99; // prob to obtain a good final model
	double w = 0.5; // estimation n(inliners)/n(total).  Empirical probability of extraction of an inliner
	int np = 3; //num of point necessary to find a plane
	int maxIterations = 100000;
} planeFitPar;


typedef struct mystructtag5
{
	Point3f bar;
	Point3f dir;
} PupilLocation;

// Equation of a ellipse from center, semi-axis and rotation
// https://math.stackexchange.com/questions/426150/what-is-the-general-equation-of-the-ellipse-that-is-not-in-the-origin-and-rotate
//
xcoords findXCoords(Pupil pupil, float y) {
	float h = pupil.center.x;
	float k = pupil.center.y;
	float A = pupil.angle / 180. * PI;
	float a = pupil.width() / 2.;
	float b = pupil.height() / 2.;
	float CA = cos(A);
	float SA = sin(A);
	float T1 = (y - k) * SA;
	float T2 = (y - k) * CA;

	// then the equation is
	// alfa^2 * (CA^2*b^2 + SA^2*a^2) + alfa(2*CA*T1*b^2 - 2*SA*T2*a^2) + T1^2*b^2 + T2^2*a^2-a^2*b^2 = 0  // by paper and hand
	// where alfa = (x-h).

	// quadratic formula
	float aa = CA * CA * b * b + SA * SA * a * a;
	float bb = 2 * CA * T1 * b * b - 2 * SA * T2 * a * a;
	float cc = T1 * T1 * b * b + T2 * T2 * a * a - a * a * b * b;

	// result
	xcoords xc;
	float delta = sqrt(bb * bb - 4.0 * aa * cc);
	if (delta >= 0.) {
		xc.x1 = (-bb + delta) / (2.0 * aa);
		xc.x1 += h;
		xc.x2 = (-bb - delta) / (2.0 * aa);
		xc.x2 += h;
		xc.valid = true;
	}
	else {
		// solution non valid
		xc.valid = false;
	}

	return  xc;
}

bool isCouplePupilValid(Pupil pL, Pupil pR, params par) {

	if (abs(pL.center.y - pR.center.y) > par.VerticalCenterTh)
		return false;

	// calculate the vertical height
	int hL = pL.height();
	int hR = pR.height();

	if (abs(hL - hR) > par.TotalHeightDiffTh)
		return false;

	return true;
}

// search inside a neighbour of the pixel P in horizontal range  to find the best  edge position
Point2f findBestEdge1D_raw(Mat gray, Point2f P, int half_range, int offy) {

	int px = round(P.x);
	int rows = gray.rows;
	int cols = gray.cols;

	int diff;
	int maxDiff = -1;
	int x_ = 0;
	if (((half_range + px + 1) >= cols) || ((-half_range + px) <= 0))
		return Point2f(0., 0.);

	for (int x = -half_range; x < half_range; x++) {
		int g1 = (int)gray.at<uchar>(P.y - offy, x + px + 1); // offy should be 0, here just for test 
		int g2 = (int)gray.at<uchar>(P.y - offy, x + px);
		diff = abs((int)g1 - (int(g2)));
		if (diff > maxDiff) {
			maxDiff = diff;
			x_ = x;
		}
	}
	return Point2f(x_ + px, P.y);
}


// search inside a neighbour of the pixel P in horizontal range  to find the best  edge position with a subpixel accuray
// this is the first version which is bit slow since the guaussian blur is on the entire image
std::vector<double> subPixelHorizontalGradMax_Slow(std::vector<double>* px, std::vector<int>* py, Mat* gray_uint8, double sigmax, double sigmay, int half_windowsize_search, int half_windowsize_fit)
{
	std::vector<double> out;
	Mat gray;

	gray_uint8->convertTo(gray, CV_64F);

	Mat blurred;

	GaussianBlur(gray, blurred, Size(0, 0), sigmax, sigmay); // when using null size, the support of the filter is automatically calculatyed fron sigmax and sigmay

	Mat dIdx;
	Mat dIdy;

	Sobel(blurred, dIdx, CV_64F, 1, 0); // default kernel size is 3*3
	Sobel(blurred, dIdy, CV_64F, 0, 1); // default kernel size is 3*3

	Mat gradientMagnitude;
	pow(dIdx.mul(dIdx) + dIdy.mul(dIdy), .5, gradientMagnitude);

	Mat A(2 * half_windowsize_fit + 1, 3, CV_64F);  // x^2 x 1
	Mat b(2 * half_windowsize_fit + 1, 1, CV_64F);  // I(x_i,y)

	for (int jj = -half_windowsize_fit; jj <= half_windowsize_fit; jj++)
	{
		A.at<double>(jj + half_windowsize_fit, 0) = (double)(jj * jj);
		A.at<double>(jj + half_windowsize_fit, 1) = (double)jj;
		A.at<double>(jj + half_windowsize_fit, 2) = 1.;
	}

	Mat pinvA;  //double

	// pseudoinverse to solve A*x = b in the least squares sense
	// where x = [a,b,c]' and the profile is modeled as px^2 + qx +r
	invert(A, pinvA, DECOMP_SVD);

	for (int ii = 0; ii < px->size(); ii++)
	{
		int max_location;
		;
		Mat window_to_search = gradientMagnitude(Rect(std::round((*px)[ii]) - half_windowsize_search, (*py)[ii], 2 * half_windowsize_search + 1, 1));

		double minValue;
		double maxValue;
		Point minPoint;
		Point maxPoint;
		minMaxLoc(window_to_search, &minValue, &maxValue, &minPoint, &maxPoint);
		int deltaxMax = maxPoint.x - half_windowsize_search;

		for (int jj = -half_windowsize_fit; jj <= half_windowsize_fit; jj++)
		{
			b.at<double>(jj + half_windowsize_fit, 0) = gradientMagnitude.at<double>((*py)[ii], std::round((*px)[ii]) + deltaxMax + jj);
		}
		Mat pqr = pinvA * b;
		double p = pqr.at<double>(0, 0);
		double q = pqr.at<double>(1, 0);

		out.push_back(-q / (2 * p) + std::round((*px)[ii]) + deltaxMax);
	}

	return out;
}

// search inside a neighbour of the pixel P in horizontal range  to find the best  edge position with a subpixel accuray
// this is the fast version
std::vector<double> subPixelHorizontalGradMax(std::vector<double>* px, std::vector<int>* py, Mat* gray_uint8, double sigmax, double sigmay, int half_windowsize_search, int half_windowsize_fit)
{
	int minx = std::round(*std::min_element(px->begin(), px->end())) - half_windowsize_search - half_windowsize_fit - std::ceil(5 * sigmax);
	int maxx = std::round(*std::max_element(px->begin(), px->end())) + half_windowsize_search + half_windowsize_fit + std::ceil(5 * sigmax);;
	int miny = std::round(*std::min_element(py->begin(), py->end())) - std::ceil(5 * sigmay);
	int maxy = std::round(*std::max_element(py->begin(), py->end())) + std::ceil(5 * sigmay);

	Rect region(minx, miny, maxx - minx, maxy - miny);


	std::vector<double> out;
	Mat gray_cropped;

	Mat gray_cropped_uint8 = (*gray_uint8)(region);

	gray_cropped_uint8.convertTo(gray_cropped, CV_64F);

	Mat blurred;

	GaussianBlur(gray_cropped, blurred, Size(0, 0), sigmax, sigmay); // when using null size, the support of the filter is automatically calculated fron sigmax and sigmay

	Mat dIdx;
	Mat dIdy;

	Sobel(blurred, dIdx, CV_64F, 1, 0); // default kernel size is 3*3
	Sobel(blurred, dIdy, CV_64F, 0, 1); // default kernel size is 3*3

	Mat gradientMagnitude;
	pow(dIdx.mul(dIdx) + dIdy.mul(dIdy), .5, gradientMagnitude);

	Mat A(2 * half_windowsize_fit + 1, 3, CV_64F);  // x^2 x 1
	Mat b(2 * half_windowsize_fit + 1, 1, CV_64F);  // I(x_i,y)

	for (int jj = -half_windowsize_fit; jj <= half_windowsize_fit; jj++)
	{
		A.at<double>(jj + half_windowsize_fit, 0) = (double)(jj * jj);
		A.at<double>(jj + half_windowsize_fit, 1) = (double)jj;
		A.at<double>(jj + half_windowsize_fit, 2) = 1.;
	}

	Mat pinvA;  //double

	// pseudoinverse to solve A*x = b in the least squares sense
	// where x = [a,b,c]' and the profile is modeled as px^2 + qx +r
	invert(A, pinvA, DECOMP_SVD);

	for (int ii = 0; ii < px->size(); ii++)
	{
		int max_location;
		;
		Mat window_to_search = gradientMagnitude(Rect(std::round((*px)[ii]) - half_windowsize_search - minx, (*py)[ii] - miny, 2 * half_windowsize_search + 1, 1));

		double minValue;
		double maxValue;
		Point minPoint;
		Point maxPoint;
		minMaxLoc(window_to_search, &minValue, &maxValue, &minPoint, &maxPoint);
		int deltaxMax = maxPoint.x - half_windowsize_search;

		for (int jj = -half_windowsize_fit; jj <= half_windowsize_fit; jj++)
		{
			b.at<double>(jj + half_windowsize_fit, 0) = gradientMagnitude.at<double>((*py)[ii] - miny, std::round((*px)[ii]) + deltaxMax + jj - minx);
		}
		Mat pqr = pinvA * b;
		double p = pqr.at<double>(0, 0);
		double q = pqr.at<double>(1, 0);

		out.push_back(-q / (2 * p) + std::round((*px)[ii]) + deltaxMax);
	}

	return out;
}

//
// go through the left and right border of the pupil and save the coupled points into a vector.
//
//  mode = 0.  Use intersection between ellipses and horizontal line  y = const
//  mode = 1.  search into -range : + range for the biggest difference in absolute values. Starting from  mode = 0 result.
//  mode = 2.  search similar to mode 1 but with interpolation and subpixel accuracy.
//  offy is the offset in y in the  right image. This should be always zero if calibration is good. Here used just for test.
vector<couple> findCouples(Mat gray, Pupil pL, Pupil pR, params par, int mode, int offy) {
	vector<couple> v;

	if (mode == 2) // subpixel
	{
		std::vector<double> x_;
		std::vector<int> y_;

		double sigmax = 1.; // of the Gaussian filter emplyed for computing the gradient
		double sigmay = 1.; // of the Gaussian filter emplyed for computing the gradient
		int half_windowsize_search = 5; //for searching the maximum along x
		int half_windowsize_fit = 3; //for for fitting the parabola

		// preparing the x_ and y_ vectors
		for (int yy = pL.center.y - par.YRange; yy < (pL.center.y + par.YRange); yy += par.YStep) {
			xcoords xcL = findXCoords(pL, yy); // from analitic intersection
			xcoords xcR = findXCoords(pR, yy);

			if (xcL.valid && xcR.valid) { // they should be already valid because tested before this function call

				// points in the left border of the pupil  (not left image)
				couple co;
				co.P_L = Point2f(xcL.x1, (float)yy); // left  image  left border
				co.P_R = Point2f(xcR.x1, (float)yy); // right image  left border
				x_.push_back(co.P_L.x);
				y_.push_back(co.P_L.y);
				x_.push_back(co.P_R.x);
				y_.push_back(co.P_R.y - offy);

				// points in the right border of the pupil  (not right image)
				co.P_L = Point2f(xcL.x2, (float)yy); // left image  right border
				co.P_R = Point2f(xcR.x2, (float)yy); // right image  right border
				x_.push_back(co.P_L.x);
				y_.push_back(co.P_L.y);
				x_.push_back(co.P_R.x);
				y_.push_back(co.P_R.y - offy);

			}
		}

		// find all subpixel in unique call
		vector<double> xsubpixel = subPixelHorizontalGradMax(&x_, &y_, &gray, sigmax, sigmay, half_windowsize_search, half_windowsize_fit);

		// reshaping the results
		for (int i = 0; i < x_.size(); i += 4) {
			couple co;
			co.P_L = Point2f(xsubpixel[i], (float)y_[i]); // left  image  left border
			co.P_R = Point2f(xsubpixel[i + 1], (float)y_[i + 1] + offy); // right image  left border
			v.push_back(co);
			co.P_L = Point2f(xsubpixel[i + 2], (float)y_[i + 2]); // left image  right border
			co.P_R = Point2f(xsubpixel[i + 3], (float)y_[i + 3] + offy); // right image  right border
			v.push_back(co);
		}
		return v;
	}

	//
	// mode = 0 and mode = 1
	//
	for (int yy = pL.center.y - par.YRange; yy < (pL.center.y + par.YRange); yy += par.YStep) {
		xcoords xcL = findXCoords(pL, yy);
		xcoords xcR = findXCoords(pR, yy);

		int range_ = 5;

		if (xcL.valid && xcR.valid) { // they should be already valid because tested before this function call

			// point in the left border of the pupil  (not left image)
			couple co;
			co.P_L = Point2f(xcL.x1, (float)yy); // from analitic intersection
			co.P_R = Point2f(xcR.x1, (float)yy);
			if (mode == 0)
				v.push_back(co);
			else if (mode == 1) {
				Point2f P_L = findBestEdge1D_raw(gray, co.P_L, range_, 0); // from raw search in the +,- range along horizontal
				Point2f P_R = findBestEdge1D_raw(gray, co.P_R, range_, offy);
				couple co2;
				co2.P_L = P_L;
				co2.P_R = P_R;
				v.push_back(co2);
			}
			else if (mode == 2) { // subpixel accuracy 


			}

			// point in the right border of the pupil  (not right image)
			co.P_L = Point2f(xcL.x2, (float)yy);
			co.P_R = Point2f(xcR.x2, (float)yy);
			if (mode == 0)
				v.push_back(co);
			else if (mode == 1) {
				Point2f P_L = findBestEdge1D_raw(gray, co.P_L, range_, 0);
				Point2f P_R = findBestEdge1D_raw(gray, co.P_R, range_, offy);
				couple co2;
				co2.P_L = P_L;
				co2.P_R = P_R;
				v.push_back(co2);
			}


		}
	}
	return v;
}

int readCalibFile(Mat &CM1, Mat &CM2, Mat &D1, Mat &D2, Mat &R1, Mat &R2, Mat &P1, Mat &P2) {

	FileStorage fs;
	fs.open("C://Users//Andrea Perissinotto//Desktop//acute_rats_videos//2021-9-9_10-45-8//stereocalib.yml", FileStorage::READ);
	//fs.open("E://SISSA_WORK//experiment_2021-7-6-10-41-32//stereocalib.yml", FileStorage::READ);
	if (fs.isOpened()) {
		fs["CM1"] >> CM1;
		fs["CM2"] >> CM2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs.release();
	}
	else {
		cout << "file params stereocalib  not found or not valid." << endl;
		return -1;
	}

	return 1;
}


//Stereo 2D points to 3D world points
//The origin of the world coordinate system is the left camera origin.
vector<Point3f> triangulatePointsPupil(vector<couple> Couples, Mat& CM1, Mat& CM2, Mat& D1, Mat& D2, Mat& R1, Mat& R2, Mat& P1, Mat& P2, int widthIm) {

	// Transforms points in both camera images to real world coordinates
	vector<Point2f> undistCornerPointsBuf, undistCornerPointsBufSecondary;

	int N = Couples.size();
	Mat imagePoints1(N, 2, CV_32F);
	Mat imagePoints2(N, 2, CV_32F);

	for (int i = 0; i < N; i++) {
		imagePoints1.at<float>(i, 0) = Couples[i].P_L.x;
		imagePoints1.at<float>(i, 1) = Couples[i].P_L.y;
		imagePoints2.at<float>(i, 0) = Couples[i].P_R.x - widthIm;
		imagePoints2.at<float>(i, 1) = Couples[i].P_R.y;
	}

	undistortPoints(imagePoints1, undistCornerPointsBuf, CM1, D1, R1, P1);
	undistortPoints(imagePoints2, undistCornerPointsBufSecondary, CM2, D2, R2, P2);

	Mat homogenPoints(4, N, CV_32F);

	triangulatePoints(P1, P2, undistCornerPointsBuf, undistCornerPointsBufSecondary, homogenPoints);

	vector<Point3f> worldPoints(N);

	// Output of triangulate points are in homogenous coordinates, convert to cartesian
	for (int i = 0; i < homogenPoints.cols; i++) {
		float scale = homogenPoints.at<float>(3, i) != 0.f ? homogenPoints.at<float>(3, i) : 1.f;
		worldPoints.at(i).x = homogenPoints.at<float>(0, i) / scale;
		worldPoints.at(i).y = homogenPoints.at<float>(1, i) / scale;
		worldPoints.at(i).z = homogenPoints.at<float>(2, i) / scale;
	}

	return worldPoints;

}

// plot 3D point in the space
void plot3D(const vector<Point3f>& Pts, vector<double>& bestDist, PupilLocation result) {
	//
	// Vis requires recompilation of opencv and other operations, now here I simply save the coords,  then using matlab we can plot.
	//
	ofstream myfile;
	myfile.open("C:\\Users\\Andrea Perissinotto\\Desktop\\Points\\Points3D_5.txt");
	if (!myfile) {
		cout << "NotWorking" << endl;
	}
	else {
		cout << "All is well!" << endl;
	}
	for (int i = 0; i < Pts.size(); i++) {
		myfile << Pts[i].x << ", " << Pts[i].y << ", " << Pts[i].z << ", " << bestDist[i] << "\n";
	}
	myfile << result.bar.x << ", " << result.bar.y << ", " << result.bar.z << ", " << 0. << "\n"; // penulitma riga baricentro
	myfile << result.dir.x << ", " << result.dir.y << ", " << result.dir.z << ", " << 0. << "\n"; // penulitma riga direzione
	myfile.close();
}

// find a 3D plane passing nearby the points,  ransac
PupilLocation findPlane(const vector<Point3f> &Pts, planeFitPar Par) {

	double iterazioni = log(1.0 - Par.probGood) / log(1.0 - pow(Par.w, Par.np));
	double stditer = sqrt(1.0 - pow(Par.w, Par.np)) / pow(Par.w, Par.np);
	iterazioni = (int)ceil(iterazioni + 2 * stditer);

	int N = Pts.size();
	vector<int> index;
	for (int i = 0; i < N; i++)
		index.push_back(i);

	int maxIn = -100000000; // to be always true at the beginning
	Point3f bestvn;
	vector<int> bestIndexes;
	vector<double> bestDist;
	for (int t = 0; t < iterazioni; t++) {
		vector<int> permutation(index);
		random_shuffle(permutation.begin(), permutation.end());

		Point3f X1 = Pts[permutation[0]];
		Point3f X2 = Pts[permutation[1]];
		Point3f X3 = Pts[permutation[2]];

		Point3f vn = (X2 - X1).cross(X3 - X1);
		double n = cv::norm(vn);
		if (n > 0.000001)
			vn = vn / n;
		else
			continue;

		// find points to plane distance and those who are less than the threshold
		vector<double> dist;
		vector<int> inIndexes;
		for (int i = 0; i < N; i++) {
			double d = (Pts[i] - X1).dot(vn);
			dist.push_back(d);
			if (abs(d) < Par.distTh) inIndexes.push_back(i);
		}
		int Numinliners = inIndexes.size();

		if (Numinliners > maxIn) {
			maxIn = Numinliners;
			bestvn = vn;
			bestIndexes = inIndexes;
			bestDist = dist;
		}
	}

	float score = (float)maxIn / N;

	// least square refinement
	double mx = 0.;
	double my = 0.;
	double mz = 0.;
	vector<Point3f> PtsIn;  // create the vector of inliers
	for (int i = 0; i < bestIndexes.size(); i++) {
		//		PtsIn.push_back(Pts[bestIndexes[i]]);
		mx += Pts[bestIndexes[i]].x;
		my += Pts[bestIndexes[i]].y;
		mz += Pts[bestIndexes[i]].z;
	}
	mx = mx / bestIndexes.size();
	my = my / bestIndexes.size();
	mz = mz / bestIndexes.size();

	//     A=[x',y',z'];  model ax+by+cz = 0   , [U, S, V] = svd(A);
	Mat A(bestIndexes.size(), 3, CV_32F);
	for (int i = 0; i < bestIndexes.size(); i++) {
		A.at<float>(i, 0) = Pts[bestIndexes[i]].x - mx;
		A.at<float>(i, 1) = Pts[bestIndexes[i]].y - my;
		A.at<float>(i, 2) = Pts[bestIndexes[i]].z - mz;
	}

	Mat w, u, vt;
	SVD::compute(A, w, u, vt); //w = singular values, u = left sing vectors, vt= transpose of right sing vectors

	int idxMin = 0; // index of the minor singular value 
	if ((w.at<float>(0, 0) < w.at<float>(1, 0)) && (w.at<float>(0, 0) < w.at<float>(2, 0))) idxMin = 0;
	if ((w.at<float>(1, 0) < w.at<float>(0, 0)) && (w.at<float>(1, 0) < w.at<float>(2, 0))) idxMin = 1;
	if ((w.at<float>(2, 0) < w.at<float>(0, 0)) && (w.at<float>(2, 0) < w.at<float>(1, 0))) idxMin = 2;


	//double dx = vt.at<float>(0, idxMin);
	//double dy = vt.at<float>(1, idxMin);
	//double dz = vt.at<float>(2, idxMin);
	double dx = vt.at<float>(idxMin, 0);
	double dy = vt.at<float>(idxMin, 1);
	double dz = vt.at<float>(idxMin, 2);
	if (dz > 0) {
		dx = -dx; dy = -dy; dz = -dz;
	}

	PupilLocation result;
	result.bar = Point3f(mx, my, mz);
	result.dir = Point3f(dx, dy, dz);

	plot3D(Pts, bestDist, result);

	return result;
}

// callback degli slider. In realta non serve a me, basta che esista per essere passata alla funzione di creazione dello slider.
// Quindi non fa niente.
void on_trackbarSig(int par, void*)
{
	//float p = (double)sigma_slider / alpha_slider_max;  // cosi p va da 0 a 1
	//sigma = p * 10.; // cioe adattiamo il range da 0 al massimo 
	//if (sigma < 1.0) sigma = 1.0; // con eventuali correzioni finali

	int p = 1;
}

//
// Parametri dell'applicazione
//
float FactorResize = .5; // Normale
float sigma = 2; // DoG sigma centrale.  Sperimentalmente un buon sigma dovrebbe essere  D/2 (D diametro del cerchio)
//enum MODES { MODE_NULL = 0, MODE_DONE, MODE_CANCEL, MODE_NEXT };
double actualFrRate = 0.;

// just to create the first time the file, then it can be modified by hand and just read.
void write_params() {
	FileStorage fs;
	fs.open("settings_PDetector.yml", FileStorage::WRITE);

	// alignment and PuRe
	float userMinPupilDiameterPx = 20.0;
	float userMaxPupilDiameterPx = 80.0;

	float TotalHeightDiffTh = (float)10.0; // pixels
	float VerticalCenterTh = (float)3.0; // pixels
	int YRange = (int)20; // half of the entire Y search range 
	int YStep = 2; // step sampling in the Y direction
	bool ShowEdges = false;

	// ransac
	double distTh = 0.02; // distance from the plane to be considered inliner.
	double probGood = 0.99; // prob to obtain a good final model
	double wRatio = 0.5; // estimation n(inliners)/n(total).  Empirical probability of extraction of an inliner
	int np = 3; //num of point necessary to find a plane
	int maxIterations = 100000;

	int half_x_search = 5; // range of search aroud the pixel edge
	int half_x_peak = 3;  // used in subpixel

	float nonEdgePixRatio = 0.7; // 19.01.2022

	if (fs.isOpened()) {
		fs << "userMinPupilDiameterPx" << userMinPupilDiameterPx
			<< "userMaxPupilDiameterPx" << userMaxPupilDiameterPx
			<< "TotalHeightDiffTh" << TotalHeightDiffTh
			<< "VerticalCenterTh " << VerticalCenterTh
			<< "YRange" << YRange
			<< "YStep" << YStep
			<< "ShowEdges" << ShowEdges
			<< "distTh" << distTh
			<< "probGood" << probGood
			<< "wRatio" << wRatio
			<< "np" << np
			<< "maxIterations" << maxIterations
			<< "halfXEdgeRange" << half_x_search
			<< "halfXPeak" << half_x_peak
			<< "nonEdgePixRatio" << nonEdgePixRatio;// 19.01.2022
	}
	fs.release();
}

int read_params(params& p, planeFitPar& pf) {

	FileStorage fs;
	string DataPath = "";
	fs.open(DataPath + "\settings_PDetector.yml", FileStorage::READ);
	if (fs.isOpened()) {

		fs["userMinPupilDiameterPx"] >> p.userMinPupilDiameterPx;
		fs["userMaxPupilDiameterPx"] >> p.userMaxPupilDiameterPx;

		fs["TotalHeightDiffTh"] >> p.TotalHeightDiffTh; // the ellipses must have the same height, or similar, if the heights differs mora than this the couple is not valid
		fs["VerticalCenterTh"] >> p.VerticalCenterTh; // the centers shoud be at the same height, this threshold check for validity
		fs["YRange"] >> p.YRange; // for (int yy = pL.center.y - par.YRange; yy < (pL.center.y + par.YRange); yy += par.YStep)
		fs["YStep"] >> p.YStep;
		fs["ShowEdges"] >> p.ShowEdges;
		fs["distTh"] >> pf.distTh; // th dist from the fitted plane
		fs["probGood"] >> pf.probGood;
		fs["wRatio"] >> pf.w; // estimation n(inliners)/n(total).  Empirical probability of extraction of an inliner
		fs["np"] >> pf.np;
		fs["maxIterations"] >> pf.maxIterations;
		fs["halfXEdgeRange"] >> p.half_x_search; // range for edge localization around the  intersection ellipse/horizontal line
		fs["halfXPeak"] >> p.half_x_peak; // range for searching the peak with subpixel precision
		fs["nonEdgePixRatio"] >> p.nonEdgePixRatio; // 19.01.2022

		fs.release();
	}
	else {
		cout << "settings path not valid." << endl;
		return -1;
	}

	return 1;

}

void printMatrix(int n, int m, Mat &M, string title){
	cout << endl << "--- " << title << " --- " << endl;
	for (int k1 = 0; k1 < n; k1++) {
		for (int k2 = 0; k2 < m; k2++)
			cout << M.at<double>(k1, k2) << "\t";
		cout << endl;
	}
}


//
// PLOT  functions
//
bool plotPupilBorder(Mat& view, Pupil pupil_, bool showPupilCenter) {
	bool continue_ = false;

	if (pupil_.valid(-2.0)) {
		ellipse(view, pupil_, cv::Scalar(0, 0, 255), 1);
		if (showPupilCenter)
			circle(view, pupil_.center, 1, CV_RGB(255, 0, 0), 3);
	}
	else {
		putText(view, "NO PUPIL LEFT FOUND", cv::Point(static_cast<int>(0.25 * view.cols),
			static_cast<int>(0.25 * view.rows)), cv::FONT_HERSHEY_PLAIN, 4, cv::Scalar(255, 0, 255), 4);
		continue_ = true;
	}
	return continue_;
}


//
// PLOT  vector of gaze direction
//
void plotGazeDirection(Mat& view, PupilLocation& pLoc, Mat P1, Mat P2) {

	int rows = view.rows;
	int cols = view.cols;

	const float L = 1.; //cm  length of the line
	vector<Point3f> P3D;
	P3D.push_back(Point3f(pLoc.bar.x, pLoc.bar.y, pLoc.bar.z));
	float tipx = pLoc.bar.x + pLoc.dir.x * L;
	float tipy = pLoc.bar.y + pLoc.dir.y * L;
	float tipz = pLoc.bar.z + pLoc.dir.z * L;
	P3D.push_back(Point3f(tipx, tipy, tipz));
	// rvecs e tvecs at zero, we suppose that the sdr in on the camera sensor
	Mat rvecs(3, 1, DataType<double>::type, 0.f);
	Mat tvecs(3, 1, DataType<double>::type, 0.f);
	Mat distCoeffsZero(5, 1, DataType<double>::type, 0.f);
	std::vector<Point2f> P2D;

	//
	// Left image , draw line of the sight of view
	// 
	// prepare the matrix  internals 3x3 from P1
	Mat internals(3, 3, DataType<double>::type, 0.f);;
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			internals.at<double>(i, j) = P1.at<double>(i, j);

	cv::projectPoints(P3D, rvecs, tvecs, internals, distCoeffsZero, P2D);
	line(view, P2D.at(0), P2D.at(1), Scalar(0, 255, 0), 1, 8, 0); // R (asse X)


	//
	// Right image , draw line of the sight of view
	// 
	// prepare the matrix  internals 3x3 from P2
	for (int i = 0; i < 3; ++i)
		for (int j = 0; j < 3; ++j)
			internals.at<double>(i, j) = P2.at<double>(i, j);

	float f = P2.at<double>(0, 0);
	float Tx = P2.at<double>(0, 3) / f;
	P3D[0].x += Tx;
	P3D[1].x += Tx;

	cv::projectPoints(P3D, rvecs, tvecs, internals, distCoeffsZero, P2D);
	line(view, P2D.at(0) + Point2f(cols / 2.0), P2D.at(1) + Point2f(cols / 2.0), Scalar(0, 255, 0), 1, 8, 0);

	return;
}

// 
void plotZoom(Mat& gray, Pupil& pupil, vector<couple>& myCouples, int RL, int mL, float RFactor)
{
	// plot dello zoom sul pattern
	int center_x = pupil.center.x;
	int center_y = pupil.center.y;
	try {
		Mat zoom = gray(cv::Range(center_y - mL, center_y + mL), cv::Range(center_x - mL, center_x + mL));
		Mat zoomResized;
		int offset_x = center_x - mL;
		int offset_y = center_y - mL;
		//float RFactor = 8.0;
		resize(zoom, zoomResized, Size(int(zoom.cols * RFactor), int(zoom.rows * RFactor)), 0., 0., INTER_LINEAR);
		cvtColor(zoomResized, zoomResized, cv::COLOR_GRAY2BGR);

		if (RL == 0)
			for (int tt = 0; tt < myCouples.size(); tt++) {
				//notare che si aggiunge mezzo pixel per tenere conto del centro del pixel
				cv::Point2f PtZ = cv::Point2f(roundf((myCouples[tt].P_L.x - offset_x + .5) * RFactor), roundf((myCouples[tt].P_L.y - offset_y + .5) * RFactor));
				circle(zoomResized, PtZ, 1, Scalar(0, 255, 0), 1, 8, 0);
			}
		if (RL == 1)
			for (int tt = 0; tt < myCouples.size(); tt++) {
				//notare che si aggiunge mezzo pixel per tenere conto del centro del pixel
				cv::Point2f PtZ = cv::Point2f(roundf((myCouples[tt].P_R.x - offset_x + .5) * RFactor), roundf((myCouples[tt].P_R.y - offset_y + .5) * RFactor));
				circle(zoomResized, PtZ, 1, Scalar(0, 255, 0), 1, 8, 0);
			}


		if (RL == 0)
			imshow("Zoom L", zoomResized);
		if (RL == 1)
			imshow("Zoom R", zoomResized);
	}
	catch (...) {
		// probabilmente riquadro che esce dal range ammissibile dell'immagine
		cout << "zoom range out of image dimensions" << endl;
	}
}

void loadImagePair(Mat &img1, Mat &img2, int i, string &ims1, string &ims2) {

	stringstream ss1, ss2, ss;
	string pref("000"); int lpr = pref.length();
	ss << pref << i;
	string ss0 = ss.str();
	int len = ss0.length();
	string ssn = ss0.substr(len-lpr, lpr);
	ss1 << "left_" << ssn << ".bmp";
	ss2 << "right_" << ssn << ".bmp";

	img1 = imread(ss1.str());
	img2 = imread(ss2.str());

	ims1 = ss1.str();
	ims2 = ss2.str();
}

int StereoCalibration();
int DoStereoAVI(bool rectified);

// ---------------------------------------------------
// main
// ---------------------------------------------------
int main(int argc, char* argv[])
{
	//int check;
	//char* dirname = "C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\tutorialspoint";
	//check = _mkdir(dirname);
	
	//
	// esegue calibrazione e salva i dati.
	// 
	int res;
	cout << "Recalibrate ? 0=false  1=true " << endl;
	cin >> res;
	if (res==1) {
//	bool DoCalibration = true;
//	if (DoCalibration) {
		StereoCalibration();
	}
	

	Pylon::PylonInitialize();
	int res2;
	cout << "Fake eye snapshots? [0=false, 1=true] " << endl;
	cin >> res2;
	if (res2 == 1) {
		StereoCams myStereo;


		// Read the previous stereo calibration 
		cout << "Starting to read the stereo calibration" << endl;
		Mat CM1 = Mat(3, 3, CV_64F);
		Mat CM2 = Mat(3, 3, CV_64F);
		Mat D1 = Mat(1, 5, CV_64F);
		Mat D2 = Mat(1, 5, CV_64F);
		Mat R = Mat(3, 3, CV_64F);
		Mat T = Mat(3, 1, CV_64F);
		Mat E = Mat(3, 3, CV_64F);
		Mat F = Mat(3, 3, CV_64F);
		Mat R1 = Mat(3, 3, CV_64F);
		Mat R2 = Mat(3, 3, CV_64F);
		Mat P1 = Mat(3, 4, CV_64F);
		Mat P2 = Mat(3, 4, CV_64F);
		FileStorage fs("stereocalib.yml", FileStorage::READ);
		fs["CM1"] >> CM1;
		fs["CM2"] >> CM2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["R"] >> R;
		fs["T"] >> T;
		fs["E"] >> E;
		fs["F"] >> F;
		fs.release();
		cout << "Done " << endl;

		// check if everything was fine during constructor
		if (myStereo.CamerasReady == false)
			return -1;
		int res1;
		res1 = myStereo.StartCaptureCameras();
		if (res1 < 0) return res;

		myStereo.SetViewOn();

		char reskey = 0;
		double ms;

#ifdef SAVEONTRIGGER
		cout << endl << "A video chunk is saved at each trigger, press q to quit." << endl;
		reskey = myStereo.GetFrames(FactorResize, ms); // impone il factor resize e le dimensioni immagine
#else
		//
		// Commentare queste righe per entrare subito in modalità acquisizione senza aspettare il tasto
		// 
		cout << endl << "Press  s  to start saving the .AVI,  press q to quit." << endl;
		while ((reskey != 's') && (reskey != 'q')) {
			reskey = myStereo.GetFrames(FactorResize, ms);
		}
		if (reskey == 'q') {
			myStereo.StopCaptureCameras();
			return -1;
		}
		//
		// FINE : Commentare queste righe per entrare subito in modalità acquisizione senza aspettare il tasto
		// 
#endif
		string pre = "experiment_"; string suf = "_fake_eye";
		std::string pathToDirectory = addTimeDateToString(pre, suf);
		//char * pathArray = new char[pathToDirectory.size() + 1];
		//std::copy(pathToDirectory.begin(), pathToDirectory.end(), pathArray);
		//pathArray[pathToDirectory.size()] = '\0'; // don't forget the terminating 0
		//for (int x = 0; x < sizeof(pathArray); x++) {
		//	pathArray[x] = pathToDirectory[x];
		//}
		
		// copying the contents of the
		// string to char array
		//strcpy(char_array, s.c_str());
		int check = 0;
		int cat_s = 1;
		std::string dirname = "C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\";
		dirname += pathToDirectory;
		//char * dirname = "C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\";
		//cat_s = strcat_s(dirname, sizeof(dirname), pathToDirectory.c_str());
		check = _mkdir(dirname.c_str());
		//
		// salva file AVI  componendo in orizzontale le due immagini
		//
		string prefix = "video_"; string suffix = ".avi";
		string fileToSave = addTimeDateToString(prefix, suffix);
		cout << "File to be saved : " << fileToSave << endl;
		fileToSave = dirname + "\\" + fileToSave;
		//string prefix = "experiment_"; string suffix = "_fake_eye";
		//string fileToSave = addTimeDateToString(prefix, suffix);
		//cout << "File to be saved : " << fileToSave << endl;
		//string pathToDirectory = "C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\" + fileToSave;
		//bool checkDir = CreateDirectory(pathToDirectory.c_str());

		VideoWriter outputVideo; // Open the output
		int w = myStereo.view0.size().width;
		int h = myStereo.view0.size().height;
		int fps = 1;
		Size S(2 * w, h);
		//cv::VideoWriter::fourcc('X', 'V', 'I', 'D')
		//	outputVideo.open("outAVI.avi", -1, fps, S, true); // chiede il codec a video
		//outputVideo.open(fileToSave.c_str(), CV_FOURCC('L','A','G','S'), fps, S, true);
		//	outputVideo.open(fileToSave.c_str(), 1, fps, S, true); //  (YUV planar 4:2:0 compressione croma)
		outputVideo.open(fileToSave.c_str(), cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'), fps, S, false); // senza compressione 
																						   //outputVideo.open("outAVI.avi", CV_FOURCC('X', 'V', 'I', 'D'), fps, S, true); // NON FUNZIONA
																						   //outputVideo.open("outAVI.avi", CV_FOURCC('D', 'X', '5', '0'), fps, S, true); // NON FUNZIONA
																						   //outputVideo.open(NAME, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);

																						   //int iii = CV_FOURCC('L', 'A', 'G', 'S');

		if (!outputVideo.isOpened())
		{
			cout << "Could not open the output video for write: " << endl;
			myStereo.StopCaptureCameras();
			return -1;
		}

		Mat composite(S, CV_8UC1);

		Mat map1x = Mat(h, w, CV_32F);
		Mat map1y = Mat(h, w, CV_32F);
		Mat map2x = Mat(h, w, CV_32F);
		Mat map2y = Mat(h, w, CV_32F);
		initUndistortRectifyMap(CM1, D1, R1, P1, myStereo.view0.size(), CV_32FC1, map1x, map1y);
		initUndistortRectifyMap(CM2, D2, R2, P2, myStereo.view1.size(), CV_32FC1, map2x, map2y);
		cout << "Remapping calculated" << endl;

		int k = 0;
		myStereo.glFrame = 0;

		cout << "Press s to get a snapshot, press q to quit." << endl;
		int numSnap = 0;
		/*
		PuRe* myPuRe = new PuRe();

		params param; // param alignment and PuRe and general purpose
		planeFitPar fitParams; // ransac
		int rep = read_params(param, fitParams);

		// edge detection
		Pupil pupil_L = Pupil();
		Pupil pupil_R = Pupil();

		int it = 0;
		int sigma_slider1;
		int sigma_slider2;
		float userMinPupilDiameterPx = param.userMinPupilDiameterPx;
		float userMaxPupilDiameterPx = param.userMaxPupilDiameterPx;
		namedWindow("image", 1);
		int alpha_slider_max = 200;
		createTrackbar("min Pupil diam", "image", &sigma_slider1, alpha_slider_max, on_trackbarSig);
		createTrackbar("max Pupil diam", "image", &sigma_slider2, alpha_slider_max, on_trackbarSig);
		setTrackbarPos("min Pupil diam", "image", (int)(userMinPupilDiameterPx));
		setTrackbarPos("max Pupil diam", "image", (int)(userMaxPupilDiameterPx));

		ofstream myfile;
		myfile.open("Points3D_sp.txt");
		*/
		while (true) {

			double msecs;
			reskey = myStereo.GetSelectedFrames(FactorResize);
			//cout << "elapsed time (msec)" << msecs << endl;
			if (reskey < 0) break;
			numSnap++;
			Mat viewGray0;
			cvtColor(myStereo.view0, viewGray0, cv::COLOR_BGR2GRAY);
			Mat viewGray1;
			cvtColor(myStereo.view1, viewGray1, cv::COLOR_BGR2GRAY);
			cout << "Snapshot number " << numSnap << endl;
			// undistort
			Size S_(w, h);
			Mat UviewGray0(S_, viewGray0.type());
			Mat UviewGray1(S_, viewGray0.type());

			remap(viewGray0, UviewGray0, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
			remap(viewGray1, UviewGray1, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
			
			clock_t start1 = clock();

			// check if we succeeded
			if (viewGray0.empty()) {
				cerr << "ERROR! blank frame grabbed\n";
				break;
			}
			/*
			// update from GUI
			userMinPupilDiameterPx = float(sigma_slider1);
			userMaxPupilDiameterPx = float(sigma_slider2);

			// check validity diameters
			if (userMinPupilDiameterPx > (userMaxPupilDiameterPx + 1)) {
				CV_Assert(0);
				//userMinPupilDiameterPx = userMaxPupilDiameterPx;
				//setTrackbarPos("min Pupil diam", "image", (int)(userMinPupilDiameterPx));
			}
			//
			// find Left pupil
			//
			Rect roi(0, 0, w / 2, h);
			myPuRe->LR = (param.ShowEdges) ? 1 : 0;
			myPuRe->run(UviewGray0, roi, pupil_L, userMinPupilDiameterPx, userMaxPupilDiameterPx); // better if cropped, in this case roi must be the full( half image)

			Mat viewl;
			cvtColor(UviewGray0, viewl, cv::COLOR_GRAY2BGR);

			bool showPupilCenter = true;
			if (plotPupilBorder(viewl, pupil_L, showPupilCenter))
				continue;
			
			//
			// find Right pupil
			//
			myPuRe->LR = (param.ShowEdges) ? 2 : 0;
			myPuRe->run(UviewGray1, roi, pupil_R, userMinPupilDiameterPx, userMaxPupilDiameterPx);
			
			Mat viewr;
			cvtColor(UviewGray1, viewr, cv::COLOR_GRAY2BGR);

			int offx = w / 2;
			int offy = 0;
			pupil_R.center.x += offx;
			pupil_R.center.y += offy; // because the images are not correctly aligned since the calibration is not good. It should be 0. ( x Sogand )
			if (plotPupilBorder(viewr, pupil_R, showPupilCenter))
				continue;

			imshow("left", viewl);
			imshow("right", viewr);
			char key = pollKey();
			
			// find couples of points in the pupil  between left and right cameras
			bool CoupleValid = isCouplePupilValid(pupil_L, pupil_R, param);
			if (CoupleValid == false) {
				cout << "pupil couple not valid." << endl;
				continue;
			}


			// extract couple of points to be converted in 3D points
			int mode = 2; // look at the function definition
			vector<couple> myCouples = findCouples(myStereo.view0, pupil_L, pupil_R, param, mode, offy);

			// finds the 3D points
			vector<Point3f> Pupil3D = triangulatePointsPupil(myCouples, CM1, CM2, D1, D2, R1, R2, P1, P2, w);

			// find the plane in 3D of the pupil border pixels
			planeFitPar fitParams;
			PupilLocation pLoc = findPlane(Pupil3D, fitParams);


			// plotZoom(gray, pLoc, myCouples, 0=left, 1= right, area from center, rescaling factor);
			if (param.ShowEdges) {
				plotZoom(viewl, pupil_L, myCouples, 0, 50, 4.0);
				plotZoom(viewr, pupil_R, myCouples, 1, 50, 4.0);
			}

			plotGazeDirection(viewl, pLoc, P1, P2);
			imshow("left", viewl);
			imshow("right", viewr);

			key = pollKey();

			clock_t start2 = clock();
			double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
			double actualFrRate = 1 / elapsed_secs;
			cout << "FPS " << actualFrRate << endl;
			myfile << pLoc.bar.x << ", " << pLoc.bar.y << ", " << pLoc.bar.z << ", " << pLoc.dir.x << ", " << pLoc.dir.y << ", " << pLoc.dir.z << "\n";
			*/
			//// 

			// save the couple of images
			Mat roi1(composite, Rect(0, 0, w, h));
			UviewGray0.copyTo(roi1);
			Mat roi2(composite, Rect(w, 0, w, h));
			UviewGray1.copyTo(roi2);

			//imshow("composito", composite);	waitKey(1);
			outputVideo.write(composite);

			k++;
		}
		//myfile.close();
		myStereo.StopCaptureCameras();
		outputVideo.release();
	}
	else {
		//
		// Apre flusso stereo e salva .AVI
		// true rettifica anche le immagini.
		DoStereoAVI(true);

		//ReadAvi("G:\\Eye Tracker\\Acquisizioni 20150916\\video_2015-9-16_15-35-3.avi");
		//ReadAvi("G:\\Eye Tracker\\Acquisizioni 20150916\\video_2015-9-16_15-0-22.avi");
		Pylon::PylonTerminate();
	}
	return 0;
}

int StereoCalibration(){


#ifdef FROMAVIFILE2
//	//------------------------------------------------------------
	// Prepara array di immagini leggendo un file AVI
	//------------------------------------------------------------
	VideoCapture MyCapture;
	vector<Mat> vecImages(5200);
	Mat iFrame;
//	bool isOpen =  MyCapture.open("C:\\Users\\Walter Vanzella\\Documents\\Visual Studio 2013\\Projects\\SaveAVI\\SaveAVI\\fc2_save_2014-08-26-103306-0000.avi");
	bool isOpen = MyCapture.open("G:\\Head Tracking\\checkboard.avi");
	bool uscita = true;
	int ii = 0;
	while (uscita) {
		uscita = MyCapture.grab();
		uscita = MyCapture.retrieve(iFrame);
		if (uscita) {
			vecImages[ii] = Mat();
			iFrame.copyTo(vecImages[ii]);
			ii++;
		}
	}
	int nFrames = ii;s

#endif




#ifdef FROMCAMERA



//#ifdef RISELEZIONA
	int res0;
	bool objpnt_flag = 0;
	cout << "Object points estimation?  [0=false, 1=true] " << endl;
	cin >> res0;
	if (res0 == 1) {
		int res1;
		objpnt_flag = 1;
		cout << "New images for object point estimation? [0=false, 1=true] " << endl;
		cin >> res1;
		if (res1 == 1) {
			Pylon::PylonInitialize();
			StereoCams myStereo;

			// check if everything was fine during constructor
			if (myStereo.CamerasReady == false)
				return -1;

			res1 = myStereo.StartCaptureCameras();
			if (res1 < 0) return res1;

			myStereo.SetViewOn();

			// delete the files left*.bmp e right*.bmp in the current directory
			cout << endl << "Deleting old object-points-snapshots." << endl;
			system("del .\\objpnt*.bmp");

			// takes snapshots of the calibration pattern
			int k = 0;
			cout << endl << "Press  s  to get a snapshot,  press q to quit." << endl;
			while (true) {
				//res = myStereo.GetFrames();
				res1 = myStereo.GetSelectedFrames(FactorResize);
				if (res1 < 0) break;

				// save the couple of images
				char nomefile[100];
				sprintf_s(nomefile, "objpnt_%03i.bmp", k);
				imwrite(nomefile, myStereo.view0);

				//sprintf_s(nomefile, "objpnt_%03i.bmp", k);
				//imwrite(nomefile, myStereo.view1);
				k++;
				cout << "Captured " << k << endl;
			}

			myStereo.StopCaptureCameras();
			Pylon::PylonTerminate();
		}
	}





	int res;
	cout << "Select new images for calibration?  [0=false, 1=true] " << endl;
	cin >> res;
	if (res == 1) {
		Pylon::PylonInitialize();
		StereoCams myStereo;

		// check if everything was fine during constructor
		if (myStereo.CamerasReady == false)
			return -1;

		res = myStereo.StartCaptureCameras();
		if (res < 0) return res;

		myStereo.SetViewOn();

		// delete the files left*.bmp e right*.bmp in the current directory
		cout << endl << "Deleting old calibration-snapshots." << endl;
		system("del .\\left*.bmp");
		system("del .\\right*.bmp");

		// takes snapshots of the calibration pattern
		int k = 0;
		cout << endl << "Press  s  to get a snapshot,  press q to quit." << endl;
		while (true) {
			//res = myStereo.GetFrames();
			res = myStereo.GetSelectedFrames(FactorResize);
			if (res < 0) break;

			// save the couple of images
			char nomefile[100];
			sprintf_s(nomefile, "left_%03i.bmp", k);
			imwrite(nomefile, myStereo.view0);

			sprintf_s(nomefile, "right_%03i.bmp", k);
			imwrite(nomefile, myStereo.view1);
			k++;
			cout << "Captured " << k << endl;
		}

		myStereo.StopCaptureCameras();
		Pylon::PylonTerminate();
	}
//#endif
#endif

	//-------------------------------------------------------------------
	// Calibrazione Stereo
	//-------------------------------------------------------------------


	// This should contain the physical location of each corner
	vector<vector<Point3f> > objPoints;
	vector<vector<Point3f> > nonPlanarObjPoints;
	// The chessboard corner points in the images
	vector<vector<Point2f> > imagePoints1, imagePoints2;
	vector<Point2f> corners1, corners2;
	cv::Scalar patternEstimate;
	cv::Scalar patternEstimateSec;
	//int index = 1;
	//double *sharpnessValue1 = new double[index], *sharpnessValue2 = new double[index];
	std::vector<double> sharpnessValue1, sharpnessValue2;
	int sharpnessThreshold = 7;
	bool lowquality = false;
	// --- the size of the square is in cm

	//Calibration myCalib1(.537f ,8 ,6, "Calib1_5percento.yml"); // checkboard 5%  originale
	//Calibration myCalib2(.537f, 8, 6, "Calib2_5percento.yml"); // checkboard 5%  originale
	//Calibration myCalib1(.214f, 8, 6, "Calib1_2percento.yml"); // checkboard 2%  originale
	//Calibration myCalib2(.214f, 8, 6, "Calib2_2percento.yml"); // checkboard 2%  originale
	//Calibration myCalib1(.200f, 8, 6, "Calib1_2percento.yml"); // checkboard 2%  17.06.2021
	//Calibration myCalib2(.200f, 8, 6, "Calib2_2percento.yml"); // checkboard 2%  17.06.2021
	//Calibration myCalib1((.10889f), 8, 6, "Calib1_1percento.yml"); // checkboard 1%  originale
	//Calibration myCalib2((.10889f), 8, 6, "Calib2_1percento.yml"); // checkboard 1%  originale
	//Calibration objCalib((.10889f), 8, 6, "objCalib_1percento.yml");
	//Calibration myCalib1((.13149f), 10, 7, "Calib1_1percento.yml"); // checkboard 1%  8*11
	//Calibration myCalib2((.13149f), 10, 7, "Calib2_1percento.yml"); // checkboard 1%  8*11
	//Calibration objCalib((.13149f), 10, 7, "objCalib_1percento.yml");
	Calibration myCalib1((.0995f), 9, 6, "Calib1_1percento.yml"); // checkboard 1%  7*10
	Calibration myCalib2((.0995f), 9, 6, "Calib2_1percento.yml"); // checkboard 1%  7*10
	Calibration objCalib((.0995f), 9, 6, "objCalib_1percento.yml");
	if (objpnt_flag)
	{

		objCalib.SetImages(".\\", "left*.bmp");
		objCalib.ObjectPointsEstimation(3.0);
		myCalib1.SetImages(".\\", "left*.bmp");
		cout << "Start single camera calibration..." << endl;
		myCalib1.Calibrate(3.0, true);
		cout << "Done" << endl;
		myCalib2.SetImages(".\\", "right*.bmp");
		cout << "Start single camera calibration..." << endl;
		myCalib2.Calibrate(3.0, true);
		cout << "Done" << endl;
	}
	else
	{
		myCalib1.SetImages(".\\", "left*.bmp");
		cout << "Start single camera calibration..." << endl;
		myCalib1.Calibrate(3.0, false);
		cout << "Done" << endl;
		myCalib2.SetImages(".\\", "right*.bmp");
		cout << "Start single camera calibration..." << endl;
		myCalib2.Calibrate(3.0, false);
		cout << "Done" << endl;
	}
	Size chessboardSize(9, 6);

	// The images, which are proceeded
	Mat img1, img2;
	// The grayscale versions of the images
	Mat gray1, gray2;

	// Get the image count
	int imageCount;
	//cout << "How many images to load (for stereo calibration): " << endl;
	//cin >> imageCount;
	imageCount = myCalib2.listFiles->size();
	if (imageCount < 1) return 0;

	// The image number of the current image 
	int i = 0;
	// Whether the chessboard corners in the images were found
	bool found1 = false, found2 = false;


	// 
	int rows = 0;
	int cols = 0;
	int myType = 0;

	while (i < imageCount) {

		// Load the images
		cout << "Attempting to load image pair " << i << endl;
		string nameImg1, nameImg2;
		loadImagePair(img1, img2, i, nameImg1, nameImg2);
		cout << "Loaded image pair" << endl;

		if ((img1.data == NULL) || (img2.data == NULL)) {
			i++;
			cout << "problem with data" << endl;
			continue;
		}

		// controlla che le immagini della coppia letta siano valide. Cioè ritenute valide durante la calibrazione della singola telecamera
		if (find(myCalib1.listFiles->begin(), myCalib1.listFiles->end(), nameImg1) == myCalib1.listFiles->end()) {
			// non c'e'
			i++;
			cout << "problem with list1" << endl;
			continue;
		}

		// controlla che le immagini della coppia letta siano valide. Cioè ritenute valide durante la calibrazione della singola telecamera
		if (find(myCalib2.listFiles->begin(), myCalib2.listFiles->end(), nameImg2) == myCalib2.listFiles->end()) {
			// non c'e'
			i++;
			cout << "problem with list2" << endl;
			continue;
		}

		// Convert to grayscale images
		cvtColor(img1, gray1, cv::COLOR_BGR2GRAY);
		cvtColor(img2, gray2, cv::COLOR_BGR2GRAY);

		// Find chessboard corners
		found1 = findChessboardCorners(img1, chessboardSize, corners1, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		found2 = findChessboardCorners(img2, chessboardSize, corners2, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
		patternEstimate = cv::estimateChessboardSharpness(img1, chessboardSize, corners1);
		patternEstimateSec = cv::estimateChessboardSharpness(img2, chessboardSize, corners2);
		sharpnessValue1.push_back(patternEstimate[0]);
		sharpnessValue2.push_back(patternEstimateSec[0]);
		//index++;
		std::cout << "Avg. Sharpness" << patternEstimate[0] << ", Avg. min. brightness" << patternEstimate[1] << ", average max. brightness" << patternEstimate[3] << std::endl;
		std::cout << "Sec: Avg. Sharpness" << patternEstimateSec[0] << ", Avg. min. brightness" << patternEstimateSec[1] << ", average max. brightness" << patternEstimateSec[3] << std::endl;
		if (patternEstimate[0] > sharpnessThreshold && patternEstimateSec[0] > sharpnessThreshold) {
			lowquality = true;
		}
		
		std::cout << "found : " << found1 << "/" << found2 << endl;

		// Find corners to subpixel accuracy
		if (found1 && !lowquality) {
			cornerSubPix(gray1, corners1, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));
			drawChessboardCorners(gray1, chessboardSize, corners1, found1);
			imshow("gray 1", gray1);
			
		}
		if (found2 && !lowquality) {
			cornerSubPix(gray2, corners2, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));
			drawChessboardCorners(gray2, chessboardSize, corners2, found2);
			imshow("gray 2", gray2);
			
		}

		cv::waitKey(100);

		rows = img1.size().height;
		cols = img1.size().width;
		myType = img1.type();

		// Store corners
		if (found1 && found2 && !lowquality) {
			imagePoints1.push_back(corners1);
			imagePoints2.push_back(corners2);
			if (objpnt_flag)
			{
				/*nonPlanarObjPoints.push_back(objCalib.checkboardCorners);
				for (int i = 0; i < objCalib.checkboardCorners.size(); i++)
				{
					objCalib.checkboardCorners[i].z = 0.;
				}*/
				objPoints.push_back(objCalib.checkboardCorners);
			}
			else
			{
				objPoints.push_back(myCalib1.checkboardCorners);
			}
			cout << "Corners stored" << endl;
			i++;
		}
		// Error
		else {
			cout << "Corners not found! skip this couple." << endl;
			//return 0;
			lowquality = false;
			i++;
		}
	}

	destroyWindow("gray 1");
	destroyWindow("gray 2");

	cout << "Starting STEREO calibration..." << endl;
	Mat CM1 = Mat(3, 3, CV_64F);
	Mat CM2 = Mat(3, 3, CV_64F);
	Mat D1 = Mat(1, 5, CV_64F);
	Mat D2 = Mat(1, 5, CV_64F);
	Mat R = Mat(3, 3, CV_64F);
	Mat T = Mat(3, 1, CV_64F);
	Mat E = Mat(3, 3, CV_64F);
	Mat F = Mat(3, 3, CV_64F);

	CM1 = myCalib1.cameraMatrix;
	CM2 = myCalib2.cameraMatrix;
	D1 = myCalib1.distCoeffs;
	D2 = myCalib2.distCoeffs;

	//stereoCalibrate(objPoints, imagePoints1, imagePoints2, CM1, D1, CM2, D2, img1.size(), R, T, E, F,
	//CV_CALIB_SAME_FOCAL_LENGTH | CV_CALIB_ZERO_TANGENT_DIST, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5));

	//printMatrix(3, 3, myCalib1.cameraMatrix, "myCalib1.cameraMatrix");

//	double repStereoErr = stereoCalibrate(objPoints, imagePoints1, imagePoints2, CM1 , D1, CM2, D2,
//		img1.size(), R, T, E, F, CV_CALIB_FIX_INTRINSIC, cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 1000, 1e-6));
	double repStereoErr = stereoCalibrate(objPoints, imagePoints1, imagePoints2, CM1, D1, CM2, D2,
		cv::Size(cols,rows), R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_FOCAL_LENGTH, TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 1e-6));
	// cv::CALIB_FIX_INTRINSIC + 
	// cv::CALIB_USE_INTRINSIC_GUESS +
	// + cv::CALIB_USE_EXTRINSIC_GUESS
	// + cv::CALIB_ZERO_TANGENT_DIST
	cout << "------------------------------------------------------------" << endl;
	cout << "Re-projection error reported by stereoCalibrate: " << repStereoErr << endl;
	cout << "------------------------------------------------------------" << endl;

	printMatrix(3, 3, R, "R");
	printMatrix(3, 1, T, "T");
	//cout << "Intrinsic pars Camera 1" << endl;
	//for (int k1 = 0; k1 < 3; k1++)
	//	cout << myCalib1.cameraMatrix.at<double>(k1, 0) << "\t" << myCalib1.cameraMatrix.at<double>(k1, 1) << "\t" << myCalib1.cameraMatrix.at<double>(k1, 2) << endl;

	cout << "Done STEREO calibration" << endl;

	cout << "Starting calc rectification..." << endl;
	Mat R1 = Mat(3, 3, CV_64F);
	Mat R2 = Mat(3, 3, CV_64F);
	Mat P1 = Mat(3, 4, CV_64F);
	Mat P2 = Mat(3, 4, CV_64F);
	Mat Q = Mat(4, 4, CV_64F);
	//stereoRectify(myCalib1.cameraMatrix, myCalib1.distCoeffs, myCalib2.cameraMatrix, myCalib2.distCoeffs, img1.size(), R, T, R1, R2, P1, P2, Q, 0);
	stereoRectify(CM1, D1, CM2, D2, img1.size(), R, T, R1, R2, P1, P2, Q, 0);
	cout << "Done calc rectification" << endl;
	printMatrix(3, 3, R1, "R1");
	printMatrix(3, 3, R2, "R2");
	printMatrix(3, 4, P1, "P1");
	printMatrix(3, 4, P2, "P2");
	printMatrix(4, 4, Q, "Q");
	
	// Computes an reprojection error in physical measures of the calibration
	// Based on the known measures of the calibration pattern, an error is calculated using the measured distance between the pattern points through the stereo triangulation
	// Example: a known chessboard pattern used for calibration has a square size of 3mm, using the stereo triangulation the system measures a distance between the corner points of the pattern
	// of 3.02mm, resulting in a measurement/reprojection error of 0.02mm
	// The following calculates an average error over all pattern points in an image
	std::vector<float> reprojErrs;
	reprojErrs.resize(imagePoints1.size());
	for (int countOfImages = 0; countOfImages < imagePoints1.size(); countOfImages++)
	{
		// Transforms points in both camera images to real world coordinates
		std::vector<cv::Point2f> undistCornerPointsBuf, undistCornerPointsBufSecondary;

		cv::undistortPoints(cv::Mat(imagePoints1[countOfImages]), undistCornerPointsBuf, CM1, D1, R1, P1);
		cv::undistortPoints(cv::Mat(imagePoints2[countOfImages]), undistCornerPointsBufSecondary, CM2, D2, R2, P2);
		
		cv::Mat homogenPoints(4, static_cast<int>(imagePoints1[countOfImages].size()), CV_32F);

		cv::triangulatePoints(P1, P2, undistCornerPointsBuf,
			undistCornerPointsBufSecondary, homogenPoints);

		std::vector<cv::Point3f> worldPoints(imagePoints1[countOfImages].size());
		float thisDistance = 0;
		// Output of triangulate points are in homogenous coordinates, convert to cartesian
		for (int i = 0; i < homogenPoints.cols; i++) {
			float scale = homogenPoints.at<float>(3, i) != 0.f ? homogenPoints.at<float>(3, i) : 1.f;
			worldPoints.at(i).x = homogenPoints.at<float>(0, i) / scale;
			worldPoints.at(i).y = homogenPoints.at<float>(1, i) / scale;
			worldPoints.at(i).z = homogenPoints.at<float>(2, i) / scale;
			thisDistance = thisDistance + worldPoints.at(i).z;
		}
		cout << "the average distance is : " << thisDistance / homogenPoints.cols << endl ;

		std::vector<double> distances;
		for (int j = 0; j<worldPoints.size() - 1;) {
			double dist = cv::norm(worldPoints[j] - worldPoints[j + 1]);
			distances.push_back(dist);
			// TODO this function doesnt actually return an error but the measured sizes, we need to calc error to square size
			if (j%chessboardSize.width == chessboardSize.width - 2) {
				j += 2;
			}
			else {
				j++;
			}
			//projectionErrorHistory.push_back(std::make_pair((i * 1000) + j, dist));
		}
		reprojErrs[countOfImages] = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
		reprojErrs[countOfImages] = reprojErrs[countOfImages] - 0.10889;
		cout << "------------------------------------------------------------" << endl;
		cout << "Re-projection world error : " << reprojErrs[countOfImages] << endl;
		cout << "------------------------------------------------------------" << endl;
	}
	cout << "Starting to store results" << endl;
	FileStorage fs("stereocalib.yml", FileStorage::WRITE);
	fs << "CM1" << CM1;
	fs << "CM2" << CM2;
	fs << "D1" << D1;
	fs << "D2" << D2;
	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;
	fs << "R1" << R1;
	fs << "R2" << R2;
	fs << "P1" << P1;
	fs << "P2" << P2;
	fs << "Q" << Q;
	fs.release();
	cout << "Done storing results" << endl;

	cout << "Starting to apply undistort" << endl;
	Mat map1x = Mat(rows, cols, CV_32F);
	Mat map1y = Mat(rows, cols, CV_32F);
	Mat map2x = Mat(rows, cols, CV_32F);
	Mat map2y = Mat(rows, cols, CV_32F);
	initUndistortRectifyMap(CM1, D1, R1, P1, cv::Size(cols, rows), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, cv::Size(cols, rows), CV_32FC1, map2x, map2y);
	cout << "Done apply undistort" << endl;


	// The rectified images
	Mat imgU1 = Mat(cv::Size(cols, rows), myType);
	Mat imgU2 = Mat(cv::Size(cols, rows), myType);

	// Show rectified images
	i = 0;
	while (i < imageCount) {

		// Load the images
		cout << "Attempting to load image pair " << i << endl;
		string dummy;
		loadImagePair(img1, img2, i, dummy,dummy);
		if ((img1.data == NULL) || (img2.data == NULL)) {
			i++;
			continue;
		}
		cout << "Loaded image pair" << endl;
		i++;

		remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		//remap(img1, imgU1, map1x, map1y, INTER_LINEAR, BORDER_DEFAULT);
		//remap(img2, imgU2, map2x, map2y, INTER_LINEAR, BORDER_DEFAULT);

		
		//cv::putText(img1, "SHARPNESS: " + std::to_string(sharpnessValue1[i-1]) + "px", cv::Point(0.1*gray1.cols, 0.9*gray1.rows), cv::FONT_HERSHEY_PLAIN, 2, lowquality ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0), 3);
		imshow("img1", img1); 
		//cv::putText(img2, "SHARPNESS: " + std::to_string(sharpnessValue2[i-1]) + "px", cv::Point(0.1*gray2.cols, 0.9*gray2.rows), cv::FONT_HERSHEY_PLAIN, 2, lowquality ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0), 3);
		imshow("img2", img2);
		imshow("rec1", imgU1);
		imshow("rec2", imgU2);

		int key = waitKey(0);
		if (key == 'q') {
			break;
		}
	}




#ifdef FROMAVIFILE
		int da = 1;
		int a = 1800;
		bool uscita;
		VideoCapture MyCapture;
	
		bool isOpen = MyCapture.open("G:\\RatVideo\\NG19_2015_5_19\\fc2_save_2015-05-19-160432-0000.avi"); // pattern ridotto


		if (isOpen == false) {
			cout << "File not found. " << endl;
			return -1;
		}


		while ((key != 'q')  && (Quit==false))
		{

			uscita = MyCapture.grab();
			uscita = MyCapture.retrieve(view);

			// calcolo frame rate
			clock_t start2 = clock();
			double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
			actualFrRate = 1 / elapsed_secs;
			start1 = start2;

			if (uscita)
				frames++;
			else
				break;



			if ((frames < da) || (frames > a))
				continue;

#endif


	return 0;
}

// generazione di un file avi con frame doppio
int	DoStereoAVI(bool rectified){
	int res;
	StereoCams myStereo;


	// Read the previous stereo calibration 
	cout << "Starting to read the stereo calibration" << endl;
	Mat CM1 = Mat(3, 3, CV_64F);
	Mat CM2 = Mat(3, 3, CV_64F);
	Mat D1 = Mat(1, 5, CV_64F);
	Mat D2 = Mat(1, 5, CV_64F);
	Mat R = Mat(3, 3, CV_64F);
	Mat T = Mat(3, 1, CV_64F);
	Mat E = Mat(3, 3, CV_64F);
	Mat F = Mat(3, 3, CV_64F);
	Mat R1 = Mat(3, 3, CV_64F);
	Mat R2 = Mat(3, 3, CV_64F);
	Mat P1 = Mat(3, 4, CV_64F);
	Mat P2 = Mat(3, 4, CV_64F);
	FileStorage fs("stereocalib.yml", FileStorage::READ);
	fs["CM1"] >> CM1;
	fs["CM2"] >> CM2;
	fs["D1"] >> D1;
	fs["D2"] >> D2;
	fs["R1"] >> R1;
	fs["R2"] >> R2;
	fs["P1"] >> P1;
	fs["P2"] >> P2;
	fs["R"] >> R;
	fs["T"] >> T;
	fs["E"] >> E;
	fs["F"] >> F;
	fs.release();
	cout << "Done " << endl;

	// check if everything was fine during constructor
	if (myStereo.CamerasReady == false)
		return -1;

	res = myStereo.StartCaptureCameras();
	if (res < 0) return res;

	myStereo.SetViewOn();

	char reskey = 0;
	double ms;

#ifdef SAVEONTRIGGER
	cout << endl << "A video chunk is saved at each trigger, press q to quit." << endl;
	reskey = myStereo.GetFrames(FactorResize,ms); // impone il factor resize e le dimensioni immagine
#else
	//
	// Commentare queste righe per entrare subito in modalità acquisizione senza aspettare il tasto
	// 
	
	cout << endl << "Press  s  to start saving the .AVI,  press q to quit." << endl;
	while ((reskey != 's') && (reskey != 'q')) {
		reskey = myStereo.GetFrames(FactorResize,ms);
	}
	if (reskey == 'q'){
		myStereo.StopCaptureCameras();
		return -1;
	}

	//
	// FINE : Commentare queste righe per entrare subito in modalità acquisizione senza aspettare il tasto
	// 
#endif


	//
	// salva file AVI  componendo in orizzontale le due immagini
	//
	string prefix = "video_"; string suffix = ".avi";
	string fileToSave = addTimeDateToString(prefix, suffix);
	cout << "File to be saved : " << fileToSave << endl;

	VideoWriter outputVideo; // Open the output
	int w = myStereo.view0.size().width;
	int h = myStereo.view0.size().height;
	int fps = 12;
	Size S(2 * w, h);
	int my_codec = cv::VideoWriter::fourcc('W', 'M', 'V', '1');

	//outputVideo.open(fileToSave.c_str(), my_codec, 30.0, S, false); // chiede il codec a video
	//outputVideo.open(fileToSave.c_str(), cv::VideoWriter::fourcc('L','A','G','S'), fps, S, true);
	//outputVideo.open(fileToSave.c_str(), -1, fps, S, false); //  (YUV planar 4:2:0 compressione croma)
	//outputVideo.open(fileToSave.c_str(), cv::VideoWriter::fourcc('D', 'I', 'B', ' '), fps, S, false); // senza compressione 
	outputVideo.open(fileToSave.c_str(), cv::VideoWriter::fourcc('H', 'F', 'Y', 'U'), fps, S, true); // senza compressione 
	//outputVideo.open(fileToSave.c_str(), cv::VideoWriter::fourcc('I', '4', '2', '0'), fps, S, false); // NON FUNZIONA
	//outputVideo.open(fileToSave.c_str(), -1, fps, S, false); // NON FUNZIONA
	//outputVideo.open("outAVI.avi", CV_FOURCC('D', 'X', '5', '0'), fps, S, true); // NON FUNZIONA
	//outputVideo.open(NAME, ex, inputVideo.get(CV_CAP_PROP_FPS), S, true);

	//int iii = CV_FOURCC('L', 'A', 'G', 'S');

	if (!outputVideo.isOpened())
	{
		cout << "Could not open the output video for write: " << endl;
		myStereo.StopCaptureCameras();
		return -1;
	}

	Mat composite(S, CV_8UC1);

	Mat map1x = Mat(h, w, CV_32F);
	Mat map1y = Mat(h, w, CV_32F);
	Mat map2x = Mat(h, w, CV_32F);
	Mat map2y = Mat(h, w, CV_32F);
	initUndistortRectifyMap(CM1, D1, R1, P1, myStereo.view0.size(), CV_32FC1, map1x, map1y);
	initUndistortRectifyMap(CM2, D2, R2, P2, myStereo.view1.size(), CV_32FC1, map2x, map2y);
	cout << "Remapping calculated" << endl;

	int k = 0;
	myStereo.glFrame = 0;

	cout << "Press a key to start recording" << endl;
	waitKey(0);
	PuRe* myPuRe = new PuRe();

	params param; // param alignment and PuRe and general purpose
	planeFitPar fitParams; // ransac
	int rep = read_params(param, fitParams);

	// edge detection
	Pupil pupil_L = Pupil();
	Pupil pupil_R = Pupil();

	int it = 0;
	int sigma_slider1;
	int sigma_slider2;
	float userMinPupilDiameterPx = param.userMinPupilDiameterPx;
	float userMaxPupilDiameterPx = param.userMaxPupilDiameterPx;
	namedWindow("image", 1);
	int alpha_slider_max = 200;
	createTrackbar("min Pupil diam", "image", &sigma_slider1, alpha_slider_max, on_trackbarSig);
	createTrackbar("max Pupil diam", "image", &sigma_slider2, alpha_slider_max, on_trackbarSig);
	setTrackbarPos("min Pupil diam", "image", (int)(userMinPupilDiameterPx));
	setTrackbarPos("max Pupil diam", "image", (int)(userMaxPupilDiameterPx));

	ofstream myfile;
	myfile.open("Points3D_sp.txt");

	while (reskey != 'q') {

		double msecs;
		reskey = myStereo.GetFrames(FactorResize, msecs);
		//cout << "elapsed time (msec)" << msecs << endl;
		if (msecs > 1000) // è trascorso piu di un secondo dall'ultimo frame
		{
			// inserisci un frame nero
			Mat nero = Mat::zeros(S, CV_8UC1);
			outputVideo.write(nero);
		}

		Mat viewGray0;
		cvtColor(myStereo.view0, viewGray0, cv::COLOR_BGR2GRAY);
		Mat viewGray1;
		cvtColor(myStereo.view1, viewGray1, cv::COLOR_BGR2GRAY);

		// undistort
		Size S_(w, h);
		Mat UviewGray0(S_, viewGray0.type());
		Mat UviewGray1(S_, viewGray0.type());

		if (rectified) {
			remap(viewGray0, UviewGray0, map1x, map1y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
			remap(viewGray1, UviewGray1, map2x, map2y, INTER_LINEAR, BORDER_CONSTANT, Scalar());
		}
		else {
			undistort(viewGray0, UviewGray0, CM1, D1);
			undistort(viewGray1, UviewGray1, CM2, D2);
			//imshow("3", UviewGray0);
			//imshow("4", UviewGray1);
		}


		clock_t start1 = clock();

		// check if we succeeded
		if (viewGray0.empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}
		// check if we succeeded
		if (viewGray1.empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}
		// save the couple of images
		Mat roi1(composite, Rect(0, 0, w, h));
		UviewGray0.copyTo(roi1);
		Mat roi2(composite, Rect(w, 0, w, h));
		UviewGray1.copyTo(roi2);

		Mat view;
		cvtColor(composite, view, cv::COLOR_GRAY2BGR);
		outputVideo.write(view);

		// update from GUI
		userMinPupilDiameterPx = float(sigma_slider1);
		userMaxPupilDiameterPx = float(sigma_slider2);

		// check validity diameters
		if (userMinPupilDiameterPx > (userMaxPupilDiameterPx + 1)) {
			CV_Assert(0);
			//userMinPupilDiameterPx = userMaxPupilDiameterPx;
			//setTrackbarPos("min Pupil diam", "image", (int)(userMinPupilDiameterPx));
		}
		//
		// find Left pupil
		//
		Rect roi(0, 0, w, h);
		myPuRe->LR = (param.ShowEdges) ? 1 : 0;
		myPuRe->nonEdgePixRatio = param.nonEdgePixRatio; // 19.01.2022
		myPuRe->run(UviewGray0, roi, pupil_L, userMinPupilDiameterPx, userMaxPupilDiameterPx); // better if cropped, in this case roi must be the full( half image)

		Mat viewl;
		cvtColor(UviewGray0, viewl, cv::COLOR_GRAY2BGR);

		bool showPupilCenter = true;
		if (plotPupilBorder(view, pupil_L, showPupilCenter))
			continue;

		//
		// find Right pupil
		//
		myPuRe->LR = (param.ShowEdges) ? 2 : 0;
		myPuRe->nonEdgePixRatio = param.nonEdgePixRatio; // 19.01.2022
		myPuRe->run(UviewGray1, roi, pupil_R, userMinPupilDiameterPx, userMaxPupilDiameterPx);

		Mat viewr;
		cvtColor(UviewGray1, viewr, cv::COLOR_GRAY2BGR);

		int offx = w;
		int offy = 0;
		pupil_R.center.x += offx;
		pupil_R.center.y += offy; // because the images are not correctly aligned since the calibration is not good. It should be 0. ( x Sogand )
		if (plotPupilBorder(view, pupil_R, showPupilCenter))
			continue;

		imshow("image", view);
		//outputVideo.write(view);
		char key = pollKey();

		// find couples of points in the pupil  between left and right cameras
		bool CoupleValid = isCouplePupilValid(pupil_L, pupil_R, param);
		if (CoupleValid == false) {
			//cout << "pupil couple not valid." << endl;
			continue;
		}


		// extract couple of points to be converted in 3D points
		int mode = 2; // look at the function definition
		vector<couple> myCouples = findCouples(composite, pupil_L, pupil_R, param, mode, offy);

		// finds the 3D points
		vector<Point3f> Pupil3D = triangulatePointsPupil(myCouples, CM1, CM2, D1, D2, R1, R2, P1, P2, w);

		// find the plane in 3D of the pupil border pixels
		planeFitPar fitParams;
		PupilLocation pLoc = findPlane(Pupil3D, fitParams);


		// plotZoom(gray, pLoc, myCouples, 0=left, 1= right, area from center, rescaling factor);
		if (param.ShowEdges) {
			plotZoom(composite, pupil_L, myCouples, 0, 70, 4.0);
			plotZoom(composite, pupil_R, myCouples, 1, 70, 4.0);
		}

		plotGazeDirection(view, pLoc, P1, P2);
		imshow("image", view);

		key = pollKey();

		clock_t start2 = clock();
		double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
		double actualFrRate = 1 / elapsed_secs;
		cout << "FPS " << actualFrRate << endl;
		myfile << pLoc.bar.x << ", " << pLoc.bar.y << ", " << pLoc.bar.z << ", " << pLoc.dir.x << ", " << pLoc.dir.y << ", " << pLoc.dir.z << "\n";
		//// 


		//imshow("composito", composite);	waitKey(1);
		//outputVideo.write(view);

		k++;
	}
	myfile.close();
	myStereo.StopCaptureCameras();
	outputVideo.release();
	
	return 0;
}

/*
// Reprojection error of the calibration
// This error describe the pixel reprojection errors, for physical measure errors see reprojectionWorldErrors
std::vector<float> StereoCameraCalibration::reprojectionErrors(const std::vector<std::vector<cv::Point3f>> &objectPoints, const std::vector<std::vector<cv::Point2f>> &f_imagePoints, std::vector<cv::Mat> m_rvecs, std::vector<cv::Mat> m_tvecs, const cv::Mat &m_cameraMatrix, const  cv::Mat &m_distCoeffs) {

	std::vector<cv::Point2f> imagePoints2;
	std::vector<float> reprojErrs;

	int i;
	double err;
	reprojErrs.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		// Reproject actual objectpoint (optimal distance ie. the checkerboard size)
		cv::projectPoints(cv::Mat(objectPoints[i]), m_rvecs[i], m_tvecs[i], m_cameraMatrix, m_distCoeffs, imagePoints2);
		// Calculate distance of reprojection and actual point on saved imagepoints
		err = cv::norm(cv::Mat(f_imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

		reprojErrs[i] = err / imagePoints2.size();
	}

	return reprojErrs;
}

// Computes an reprojection error in physical measures of the calibration
// Based on the known measures of the calibration pattern, an error is calculated using the measured distance between the pattern points through the stereo triangulation
// Example: a known chessboard pattern used for calibration has a square size of 3mm, using the stereo triangulation the system measures a distance between the corner points of the pattern
// of 3.02mm, resulting in a measurement/reprojection error of 0.02mm
// The following calculates an average error over all pattern points in an image
std::vector<float> StereoCameraCalibration::reprojectionWorldErrors(const std::vector<std::vector<cv::Point2f>> &f_imagePoints, const std::vector<std::vector<cv::Point2f>> &f_imagePointsSecondary) {

	std::vector<cv::Point2f> imagePoints2;
	std::vector<float> reprojErrs;

	int i;
	reprojErrs.resize(f_imagePoints.size());

	for (i = 0; i < (int)f_imagePoints.size(); ++i) {
		std::vector<cv::Point3f> worldPoints = convertPointsTo3D(f_imagePoints[i], f_imagePointsSecondary[i]);

		std::vector<double> distances;
		for (int j = 0; j<worldPoints.size() - 1;) {
			double dist = cv::norm(worldPoints[j] - worldPoints[j + 1]);
			distances.push_back(dist);
			// TODO this function doesnt actually return an error but the measured sizes, we need to calc error to square size
			if (j%boardSize.width == boardSize.width - 2) {
				j += 2;
			}
			else {
				j++;
			}
			//projectionErrorHistory.push_back(std::make_pair((i * 1000) + j, dist));
		}
		reprojErrs[i] = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
	}

	return reprojErrs;
}
*/