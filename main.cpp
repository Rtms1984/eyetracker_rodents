#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>

#include <vector>
#include <algorithm> // std::min_element
#include <iterator>  // std::begin, std::end


#include <opencv2/core/core.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/viz.hpp>

#include "PuRe.h"

//#define USING_CAMERA
#define USING_FILE

using namespace cv;
using namespace std;

#define PI 3.141592


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
	double distTh = 0.02; // distance from the plane to be considered inliner.
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

int isCouplePupilValid(Pupil pL, Pupil pR, params par) {

	if (abs(pL.center.y - pR.center.y) > par.VerticalCenterTh)
		return -1;

	// calculate the vertical height
	int hL = pL.height();
	int hR = pR.height();
	int out1 = pL.center.y - pL.size.height;
	int out2 = (pR.center.x - pR.size.width);
	int out3 = (pR.center.y - pR.size.height);
	int out4 = (pL.center.x - pL.size.width);
	if (abs(hL - hR) > par.TotalHeightDiffTh)
		return -2;
	if ((out1 < 0) || (out2 < 0) || (out3 < 0) || (out4 < 0))
		return -3;

	return 1;
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

int readCalibFile(Mat& CM1, Mat& CM2, Mat& D1, Mat& D2, Mat& R1, Mat& R2, Mat& P1, Mat& P2) {

	FileStorage fs;
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2021-6-29_12-4-33_fake_eye//stereocalib_org.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//acute_rats_videos//2021-9-30_12-25-5//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2021-11-2_16-29-3_fake_eye//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2021-6-10-16-26-51_fake_eye//stereocalib_org.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//acute_rats_videos//2021-8-13_15-58-39//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2021-11-3_11-27-32_fake_eye//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2021-11-3_17-27-47_fake_eye//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//EyeTracker_Basler_EdgeDetection_SubPixel//EyeTracker//EyeTracker//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//10 december//stereocalib.yml", FileStorage::READ);
	fs.open("C://Users//Andrea Perissinotto//Desktop//27 january//stereocalib.yml", FileStorage::READ);
	//fs.open("C://Users//Andrea Perissinotto//Desktop//fake_eye//experiment_2022-1-14_16-32-57_fake_eye//stereocalib.yml", FileStorage::READ);
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

	Mat homogenPoints(4, N, CV_32F);

	//undistortPoints(imagePoints1, undistCornerPointsBuf, CM1, D1, R1, P1);
	//undistortPoints(imagePoints2, undistCornerPointsBufSecondary, CM2, D2, R2, P2);

	// Not necessary to undistort !!  we are reading undistorted images.
	for (int i = 0; i < N; i++) {
		undistCornerPointsBuf.push_back(Point2f(Couples[i].P_L.x, Couples[i].P_L.y));
		undistCornerPointsBufSecondary.push_back(Point2f(Couples[i].P_R.x - widthIm, Couples[i].P_R.y));
	}
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
	myfile.open("Points3D_sp.txt");
	for (int i = 0; i < Pts.size(); i++) {
		myfile << Pts[i].x << ", " << Pts[i].y << ", " << Pts[i].z << ", " << bestDist[i] << "\n";
	}
	myfile << result.bar.x << ", " << result.bar.y << ", " << result.bar.z << ", " << 0. << "\n"; // penulitma riga baricentro
	myfile << result.dir.x << ", " << result.dir.y << ", " << result.dir.z << ", " << 0. << "\n"; // penulitma riga direzione
	myfile.close();
}

// find a 3D plane passing nearby the points,  ransac
PupilLocation findPlane(const vector<Point3f>& Pts, planeFitPar Par, vector<Point3f>& PtsOnPlane) {

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


	// save the set of good 3D points of this frame 
	ofstream my3DGoodPoints;
	my3DGoodPoints.open("Points3D_lastFrame.txt");
	for (int i = 0; i < bestIndexes.size(); i++) {
		my3DGoodPoints << (Pts[bestIndexes[i]].x - mx) << ", " << (Pts[bestIndexes[i]].y - my) << ", " << (Pts[bestIndexes[i]].z - mz) << "\n";
	}
	//	my3DGoodPoints << mx << ", " << my << ", " << mz << "\n";
	my3DGoodPoints << dx << ", " << dy << ", " << dz << "\n";
	my3DGoodPoints.close();

	PupilLocation result;
	result.bar = Point3f(mx, my, mz);
	result.dir = Point3f(dx, dy, dz);

	//plot3D(Pts, bestDist, result);
	cout << "score " << score << "     maxIn / N " << maxIn << " / " << N << "  ";

	// 12.04.2022  projection of the good points on the plane
	for (int i = 0; i < bestIndexes.size(); i++) {
		double prSc = dx * (Pts[bestIndexes[i]].x - mx) + dy * (Pts[bestIndexes[i]].y - my) + dz * (Pts[bestIndexes[i]].z - mz);
		PtsOnPlane.push_back(Point3f(Pts[bestIndexes[i]].x + dx * prSc, Pts[bestIndexes[i]].y + dy * prSc, Pts[bestIndexes[i]].z + dz * prSc));
	}

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
void plotZoom(Mat& gray, Pupil& pupil, vector<couple>& myCouples, int RL, int mL, float RFactor, vector<Point2f>& ptsOnPlane2D)
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
				circle(zoomResized, cv::Point2f(roundf((center_x - offset_x + .5) * RFactor), roundf((center_y - offset_y + .5) * RFactor)), 1, Scalar(0, 255, 0), 1, 8, 0);

				cv::Point2f Pt = cv::Point2f(roundf((ptsOnPlane2D[tt].x - offset_x + .5) * RFactor), roundf((ptsOnPlane2D[tt].y - offset_y + .5) * RFactor));
				circle(zoomResized, Pt, 1, Scalar(0, 255, 255), 1, 8, 0);
			}
		if (RL == 1)
			for (int tt = 0; tt < myCouples.size(); tt++) {
				//notare che si aggiunge mezzo pixel per tenere conto del centro del pixel
				cv::Point2f PtZ = cv::Point2f(roundf((myCouples[tt].P_R.x - offset_x + .5) * RFactor), roundf((myCouples[tt].P_R.y - offset_y + .5) * RFactor));
				circle(zoomResized, PtZ, 1, Scalar(0, 255, 0), 1, 8, 0);
				circle(zoomResized, cv::Point2f(roundf((center_x - offset_x + .5) * RFactor), roundf((center_y - offset_y + .5) * RFactor)), 1, Scalar(0, 255, 0), 1, 8, 0);

				cv::Point2f Pt = cv::Point2f(roundf((ptsOnPlane2D[tt].x - offset_x + .5) * RFactor), roundf((ptsOnPlane2D[tt].y - offset_y + .5) * RFactor));
				circle(zoomResized, Pt, 1, Scalar(0, 255, 255), 1, 8, 0);
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

// ---------------------------------------------------
// main
// ---------------------------------------------------
int main(int argc, char* argv[])
{

	Mat view;
	Mat gray;
	char key = 0;

	//write_params();

	PuRe* myPuRe = new PuRe();

#ifdef USING_FILE
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\Modified Basler\\EyeTracker\\EyeTracker\\video_2021-7-6_10-41-32.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\acute_rats_videos\\2021-9-30_12-25-5\\video_2021-9-30_12-26-46.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\acute_rats_videos\\2021-8-13_15-58-39\\video_2021-8-13_15-52-14.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\acute_rats_videos\\2021-9-21_14-26-19\\video_2021-9-21_14-27-45.avi");
//	VideoCapture cap("C:\\Users\\Walter\\source\\repos\\PupilLocalization\\outAVI.avi");
//	VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-6-29_12-4-33_fake_eye\\video_2021-6-29_12-4-33.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-6-10-16-26-51_fake_eye\\video_2021-6-10_16-26-51.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-6-17_12-20-49_fake_eye\\video_2021-6-17_12-20-49.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-11-2_16-29-3_fake_eye\\video_2021-11-2_16-29-3.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-11-3_11-27-32_fake_eye\\video_2021-11-3_11-27-32.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2021-11-3_17-27-47_fake_eye\\video_2021-11-3_17-27-47.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-1-24_10-26-58.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-1-21_10-26-19.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\fake_eye\\experiment_2022-1-14_16-32-57_fake_eye\\video_2022-1-14_16-32-57.avi");
	VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\videos\\video_2022-1-27_10-4-54.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-1-28_10-10-39.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-1-27_10-18-30.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-2-2_11-42-17.avi");
	//VideoCapture cap("C:\\Users\\Andrea Perissinotto\\Desktop\\EyeTracker_Basler_EdgeDetection_SubPixel\\EyeTracker\\EyeTracker\\video_2022-2-4_11-11-18.avi");
	if (!cap.isOpened())
	{
		cout << "Could not open the output video for write: " << endl;
		cv::waitKey(1000);
	}
	int numFrame = cap.get(CAP_PROP_FRAME_COUNT);
	cap.read(view);
#endif


#ifdef USING_CAMERA

	VideoCapture cap;
	//open the default camera using default API
   //cap.open(0);
   // OR advance usage: select any API backend
	int deviceID = 0;             // 0 = open default camera
	int apiID = cv::CAP_ANY;      // 0 = autodetect default API
								  // open selected camera using selected API
	int a1 = cv::CAP_GIGANETIX;
	int a2 = cv::CAP_VFW;

	//cap.open(0, apiID);
	// check if we succeeded
	for (int iC = 0; iC < 10000; ++iC) {
		cap.open(iC);
		if (!cap.isOpened()) {
			cerr << "ERROR! Unable to open camera id = " << iC << "\n";
			//			return -1;
		}
		else {
			deviceID = iC;
			break;
		}
	}
#endif


	params param; // param alignment and PuRe and general purpose
	planeFitPar fitParams; // ransac
	int res = read_params(param, fitParams);


	//Size S(cols, rows);
	Size S(1920, 600);
	//Size S(1280, 512);

	VideoWriter outputVideo; // Open the output
	int fps = 12;
	//outputVideo.open("output_test.avi", VideoWriter::fourcc('D', 'I', 'B', ' '), fps, S, true); // senza compressione 
	outputVideo.open("output_test_w.avi", VideoWriter::fourcc('H', 'F', 'Y', 'U'), fps, S, true); // senza compressione 


	Pupil pupil_L = Pupil();
	Pupil pupil_R = Pupil();


	Mat CM1, CM2, D1, D2, R1, R2, P1, P2;
	res = readCalibFile(CM1, CM2, D1, D2, R1, R2, P1, P2);

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
	/*cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);
	cap.read(view);*/
	while ((key != 'q') && (key != 'Q') && (numFrame != 0))
	{
		it++;
		// Get the image
		cap.read(view);
		int rows = view.rows;
		int cols = view.cols;

		clock_t start1 = clock();

		// check if we succeeded
		if (view.empty()) {
			cerr << "ERROR! blank frame grabbed\n";
			break;
		}

		numFrame--;

		cvtColor(view, gray, cv::COLOR_BGR2GRAY);

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
		Rect roi(0, 0, cols / 2, rows);
		//myPuRe->run(gray, roi, pupil_L, userMinPupilDiameterPx, userMaxPupilDiameterPx);
		Mat cropped_image = gray(Range(0, rows), Range(0, cols / 2));
		//myPuRe->run(cropped_image, pupil_L);
		myPuRe->LR = (param.ShowEdges) ? 1 : 0;
		myPuRe->nonEdgePixRatio = param.nonEdgePixRatio; // 19.01.2022
		myPuRe->run(cropped_image, roi, pupil_L, userMinPupilDiameterPx, userMaxPupilDiameterPx); // better if cropped, in this case roi must be the full( half image)

		bool showPupilCenter = true;
		if (plotPupilBorder(view, pupil_L, showPupilCenter))
			continue;

		//
		// find Right pupil
		//
		cropped_image = gray(Range(0, rows), Range(cols / 2, cols));
		myPuRe->LR = (param.ShowEdges) ? 2 : 0;
		myPuRe->nonEdgePixRatio = param.nonEdgePixRatio; // 19.01.2022
		myPuRe->run(cropped_image, roi, pupil_R, userMinPupilDiameterPx, userMaxPupilDiameterPx);
		//		myPuRe->run(cropped_image, pupil_R);

		int offx = cols / 2;
		int offy = 0;
		pupil_R.center.x += offx;
		pupil_R.center.y += offy; // because the images are not correctly aligned since the calibration is not good. It should be 0. ( x Sogand )
		if (plotPupilBorder(view, pupil_R, showPupilCenter))
			continue;


		imshow("image", view);
		key = pollKey();
		//key = waitKey(0);





		// find couples of points in the pupil  between left and right cameras
		int CoupleValid = isCouplePupilValid(pupil_L, pupil_R, param);
		if (CoupleValid != 1) {
			cout << "pupil couple not valid." << CoupleValid << endl;
			continue;
		}

		// extract couple of points to be converted in 3D points
		int mode = 2; // look at the function definition
		vector<couple> myCouples = findCouples(gray, pupil_L, pupil_R, param, mode, offy);

		// finds the 3D points
		vector<Point3f> Pupil3D = triangulatePointsPupil(myCouples, CM1, CM2, D1, D2, R1, R2, P1, P2, cols / 2);

		// find the plane in 3D of the pupil border pixels
		// and projecting the 3D points on the plane, obaining PtsOnPlane3D
		vector<Point3f> PtsOnPlane3D;
		PupilLocation pLoc = findPlane(Pupil3D, fitParams, PtsOnPlane3D);

		// project PtsOnPlane on the image
		//Mat AA = Mat::eye(3,3,CV_32F);
		//Mat rvec;
		//Rodrigues(AA, rvec); // to have a correct null rotation vector
		Mat rvec(3, 1, DataType<double>::type, 0.f);
		Mat tvec(3, 1, DataType<double>::type, 0.f);
		std::vector<Point2f> PtsOnPlane2D_1;
		std::vector<Point2f> PtsOnPlane2D_2;
		Mat A1 = P1(Rect(0, 0, 3, 3)); // takes only the internals
		Mat A2 = P2(Rect(0, 0, 3, 3)); // takes only the internals
		Mat distCoeffsZero(5, 1, DataType<double>::type, 0.f);
		projectPoints(PtsOnPlane3D, rvec, tvec, A1, distCoeffsZero, PtsOnPlane2D_1);
		projectPoints(PtsOnPlane3D, rvec, tvec, A2, distCoeffsZero, PtsOnPlane2D_2);

		// plotZoom(gray, pLoc, myCouples, 0=left, 1= right, area from center, rescaling factor);
		if (param.ShowEdges) {
			plotZoom(gray, pupil_L, myCouples, 0, 50, 4.0, PtsOnPlane2D_1);
			plotZoom(gray, pupil_R, myCouples, 1, 50, 4.0, PtsOnPlane2D_2);
		}

		plotGazeDirection(view, pLoc, P1, P2);
		imshow("image", view);


		key = pollKey();
		//key = waitKey(1000);
		//key = waitKey(0);

		clock_t start2 = clock();
		double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
		double actualFrRate = 1 / elapsed_secs;
		cout << "FPS " << actualFrRate << endl;
		myfile << pLoc.bar.x << ", " << pLoc.bar.y << ", " << pLoc.bar.z << ", " << pLoc.dir.x << ", " << pLoc.dir.y << ", " << pLoc.dir.z << "\n";
		//// 
		outputVideo.write(view);


	}
	myfile.close();
	return 0;
}




//// check correctness intersection ellipse and striaght line  y = const
//for (int yy = pupil_L.center.y - 40; yy < (pupil_L.center.y + 40); yy += 2) {
//	xcoords xcL = findXCoords(pupil_L, yy);
//	if (xcL.valid) {
//		circle(view, Point(xcL.x1, yy), 1, CV_RGB(0, 255, 0), 2);
//		circle(view, Point(xcL.x2, yy), 1, CV_RGB(0, 255, 0), 2);
//		imshow("image", view);
//		key = waitKey(0);
//	}
//} 