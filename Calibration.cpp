#include <opencv2/imgproc/imgproc.hpp>
#include "Calibration.h"
#include <Windows.h>
#include <time.h>
#include <iostream>


using namespace std;
using namespace cv;

// mio template per trovare molto velocemente la mediana (percentile = 0.5).
// o il percentile.
template <typename T> T mediana(vector<T> &v, float percentile)
{
	if ((percentile > 1.) || (percentile < 0.)) {
		return (T)0.;
	}

	vector<T> v2 = v;

	size_t n = v.size() * percentile;
	nth_element(v2.begin(), v2.begin() + n, v2.end());
	return v2[n];
}

Calibration::~Calibration() {
	delete listFiles;
}

void Calibration::writeSettings(FileStorage& fs) const                        //Write serialization for this class
{
	fs << "{" << "BoardSize_Width" << mNxSquares
		<< "BoardSize_Height" << mNySquares
		<< "Square_Size" << mDimSquare
		<< "Calibrate_FixAspectRatio" << mAspectRatio
		<< "Calibrate_AssumeZeroTangentialDistortion" << mCalibZeroTangentDist
		<< "Calibrate_FixPrincipalPointAtTheCenter" << mCalibFixPrincipalPoint
		<< "Write_outputFileName" << mNameOut
		<< "Input_FlipAroundHorizontalAxis" << mFlipVertical
		<< "}";
}
void Calibration::readSettings(const FileNode& node)                          //Read serialization for this class
{
	node["BoardSize_Width"] >> mNxSquares;
	node["BoardSize_Height"] >> mNySquares;
	node["Square_Size"] >> mDimSquare;
	node["Calibrate_FixAspectRatio"] >> mAspectRatio;
	node["Calibrate_AssumeZeroTangentialDistortion"] >> mCalibZeroTangentDist;
	node["Calibrate_FixPrincipalPointAtTheCenter"] >> mCalibFixPrincipalPoint;
	node["Write_outputFileName"] >> mNameOut;
	node["Input_FlipAroundHorizontalAxis"] >> mFlipVertical;
	interprate();
}
void Calibration::interprate()
{
	goodInput = true;
	if (mNxSquares <= 0 || mNySquares <= 0)
	{
		cerr << "Invalid Board size: " << mNxSquares << " " << mNySquares << endl;
		goodInput = false;
	}
	if (mDimSquare <= 10e-6)
	{
		cerr << "Invalid square size " << mDimSquare << endl;
		goodInput = false;
	}

	flag = 0;
	if (mCalibFixPrincipalPoint) flag |= cv::CALIB_FIX_PRINCIPAL_POINT;
	if (mCalibZeroTangentDist)   flag |= cv::CALIB_ZERO_TANGENT_DIST;
	if (mAspectRatio)            flag |= cv::CALIB_FIX_ASPECT_RATIO;

}


int Calibration::SetImages(vector<string> &vectImages){
	listFiles = new vector<string>(vectImages);
	return 1;
};

int Calibration::SetImages(string pathName, string estension){
	string est = "%s" + estension;
	listFiles = new vector<string>(get_all_files_names_within_folder(est,pathName));
	mPath = pathName;
	return 1;
}

int Calibration::Calibrate(float rejectFactor, bool objpntflag) {

	int numImages = listFiles->size();
	vector<vector<Point2f>> imagePoints;
	Size boardSize(mNxSquares, mNySquares);

	//
	// find the corners of all the images
	//
	for (int i = 0; i < numImages; i++){
		cout << (*listFiles)[i] << endl;

		Mat rgbview, view;
		view = imread(mPath + (*listFiles)[i]);

		//if (mViewOn) {
		//	imshow("Loaded Image", view);
		//	waitKey(1);
		//}

		imageSize = view.size(); 
		vector<Point2f> pointBuf;

		if (mFlipVertical)  flip(view, view, 0);
		 

		bool found;
		//found = findChessboardCorners(view, boardSize , pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//		found = findChessboardCorners(view, boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		found = findChessboardCorners(view, boardSize, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH );

		if (found) {
			Mat viewGray;
			cvtColor(view, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(viewGray, pointBuf, Size(11, 11), 	Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
			imagePoints.push_back(pointBuf);
		}
		else {
			(*listFiles)[i] = string("NotFound");
		}


		// Draw the corners.
		if (mViewOn) {
			drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
			imshow("CheckBoard Image", view);
			waitKey(1);
		}

		//cvtColor(rgbview, view, CV_BGR2GRAY);
		//medianBlur(view, view, 3);
	}

	// imposta la matrice dei par interni e dei coeff. di distorsione
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flag & cv::CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	//
	// calculates the corner positions on the checkboard
	//
	vector<cv::Point3f> nonplanarCorners;
	checkboardCorners.clear();
	vector<vector<Point3f>> objectPoints(1);
	if (objpntflag)
	{
		FileStorage fs("objpoints.yml", FileStorage::READ);
		fs["objectPoints"] >> checkboardCorners;
		fs["objectPoints"] >> nonplanarCorners;
		for (int i = 0; i < checkboardCorners.size(); i++)
		{
			checkboardCorners[i].z = 0.;
		}
	}
	else
	{
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				checkboardCorners.push_back(Point3f(float(j * mDimSquare), float(i * mDimSquare), 0));
	}
	

	// duplica corner (calibrateCamera lo necessita)
	objectPoints[0] = checkboardCorners;
	objectPoints.resize(imagePoints.size(), objectPoints[0]);
	
	
	//Find intrinsic and extrinsic camera parameters
	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);

	cout << "------------------------------------------------------------" << endl;
	cout << "Re-projection error of planar pattern reported by calibrateCamera: " << rms << endl;
	cout << "------------------------------------------------------------" << endl;
	
	if (objpntflag)
	{
		vector<vector<Point3f>> newobjectPoints(1);
		newobjectPoints[0] = nonplanarCorners;
		newobjectPoints.resize(imagePoints.size(), newobjectPoints[0]);
		rms = calibrateCamera(newobjectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag | cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
		cout << "------------------------------------------------------------" << endl;
		cout << "Re-projection error of non-planar pattern reported by calibrateCamera: " << rms << endl;
		cout << "------------------------------------------------------------" << endl;
	}
	
	// filtra le viste con  norm(discrepanze) >  rejectFactor* mediana(norm(discrepanze)) 
	vector<vector<Point2f>> imagePointsFiltered;
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, reprojErrs, rejectFactor, imagePointsFiltered);
	if (imagePointsFiltered.size() < imagePoints.size()) // ci sono state delle viste eliminate
	{
		cout << "Some bad views found. Trying to recalculate calibration " << endl;
		objectPoints.resize(imagePointsFiltered.size(), objectPoints[0]);
		rms = calibrateCamera(objectPoints, imagePointsFiltered, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flag | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
		cout << "Re-projection error reported by calibrateCamera : " << rms << endl;
		cout << "------------------------------------------------------------" << endl;
		totalAvgErr = computeReprojectionErrors(objectPoints, imagePointsFiltered, reprojErrs, rejectFactor, imagePointsFiltered);
	}

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	//cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr;

	if (ok) SaveResults();

	destroyAllWindows();

	return 1;
}

int Calibration::ObjectPointsEstimation(float rejectFactor)
{
	int numImages = listFiles->size();
	vector<vector<Point2f>> imagePoints;
	Size boardSize(mNxSquares, mNySquares);

	//
	// find the corners of all the images
	//
	for (int i = 0; i < numImages; i++) {
		cout << (*listFiles)[i] << endl;

		Mat rgbview, view;
		view = imread(mPath + (*listFiles)[i]);

		//if (mViewOn) {
		//	imshow("Loaded Image", view);
		//	waitKey(1);
		//}

		imageSize = view.size();
		vector<Point2f> pointBuf;

		if (mFlipVertical)  flip(view, view, 0);


		bool found;
		//found = findChessboardCorners(view, boardSize , pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
//		found = findChessboardCorners(view, boardSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		found = findChessboardCorners(view, boardSize, pointBuf, cv::CALIB_CB_ADAPTIVE_THRESH);

		if (found) {
			Mat viewGray;
			cvtColor(view, viewGray, COLOR_BGR2GRAY);
			cornerSubPix(viewGray, pointBuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.001));
			imagePoints.push_back(pointBuf);
		}
		else {
			(*listFiles)[i] = string("NotFound");
		}


		// Draw the corners.
		if (mViewOn) {
			drawChessboardCorners(view, boardSize, Mat(pointBuf), found);
			imshow("CheckBoard Image", view);
			waitKey(1);
		}

		//cvtColor(rgbview, view, CV_BGR2GRAY);
		//medianBlur(view, view, 3);
	}

	// imposta la matrice dei par interni e dei coeff. di distorsione
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flag & cv::CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = 1.0;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	//
	// calculates the corner positions on the checkboard
	//
	vector<vector<Point3f>> objectPoints(1);
	vector<vector<Point3f>> newObjectPoints(1);
	checkboardCorners.clear();
	for (int i = 0; i < boardSize.height; ++i)
		for (int j = 0; j < boardSize.width; ++j)
			checkboardCorners.push_back(Point3f(float(j * mDimSquare), float(i * mDimSquare), 0));

	// duplica corner (calibrateCamera lo necessita)
	objectPoints[0] = checkboardCorners;
	objectPoints.resize(imagePoints.size(), objectPoints[0]);
	newObjectPoints[0] = checkboardCorners;


	//Find intrinsic and extrinsic camera parameters
	double rmsRO = calibrateCameraRO(objectPoints, imagePoints, imageSize, boardSize.width - 1, cameraMatrix, distCoeffs, rvecs, tvecs, newObjectPoints[0], flag | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
	objectPoints.clear();
	objectPoints.resize(imagePoints.size(), newObjectPoints[0]);
	checkboardCorners.clear();
	checkboardCorners = newObjectPoints[0];
	FileStorage fs("objpoints.yml", FileStorage::WRITE);
	fs << "objectPoints" << checkboardCorners;

	cout << "------------------------------------------------------------" << endl;
	cout << "Re-projection error reported by calibrateCameraRO: " << rmsRO << endl;
	cout << "------------------------------------------------------------" << endl;
	// filtra le viste con  norm(discrepanze) >  rejectFactor* mediana(norm(discrepanze)) 
	vector<vector<Point2f>> imagePointsFiltered;
	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints, reprojErrs, rejectFactor, imagePointsFiltered);
	if (imagePointsFiltered.size() < imagePoints.size()) // ci sono state delle viste eliminate
	{
		cout << "Some bad views found. Trying to recalculate calibration " << endl;
		newObjectPoints[0].clear();
		objectPoints.resize(imagePointsFiltered.size(), objectPoints[0]);
		rmsRO = calibrateCameraRO(objectPoints, imagePointsFiltered, imageSize, boardSize.width - 1, cameraMatrix, distCoeffs, rvecs, tvecs, newObjectPoints[0], flag | cv::CALIB_USE_INTRINSIC_GUESS | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5);
		cout << "Re-projection error reported by calibrateCamera : " << rmsRO << endl;
		cout << "------------------------------------------------------------" << endl;
		objectPoints.clear();
		objectPoints.resize(imagePointsFiltered.size(), newObjectPoints[0]);
		checkboardCorners.clear();
		checkboardCorners = newObjectPoints[0];
		FileStorage fs("objpoints.yml", FileStorage::WRITE);
		fs << "objectPoints" << checkboardCorners;
		totalAvgErr = computeReprojectionErrors(objectPoints, imagePointsFiltered, reprojErrs, rejectFactor, imagePointsFiltered);
	}

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	//cout << (ok ? "Calibration succeeded" : "Calibration failed") << ". avg re projection error = " << totalAvgErr;

	if (ok) SaveResults();

	destroyAllWindows();

	return 1;
}


double Calibration::computeReprojectionErrors(const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints, vector<float>& perViewErrors, 
	float rejectFactor, vector<vector<Point2f>> &imagePointsFiltered)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	// con valori di norma piu significativi (*)
	vector<float> myPerViewErrors;
	myPerViewErrors.resize(objectPoints.size());

	Mat figura = Mat::zeros(500, 700, CV_8UC3);
	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), cv::NORM_L2); // vedi (*)
		// verifica norma
		double myNorm_ = 0.;
		//double norm_2 = 0.;
		for (int iter = 0; iter < imagePoints[i].size(); ++iter){
			double dx = imagePoints[i][iter].x - imagePoints2[iter].x;
			double dy = imagePoints[i][iter].y - imagePoints2[iter].y;
			myNorm_ += sqrt(dx*dx + dy*dy); // (*) questa norma ha piu significato. E' la somma delle distanze
			//norm_2 += dx*dx + dy*dy;  // (*) cv::norm fa questo e alla fine  norm_2 = sqrt(norm_2);
		}

		int n = (int)objectPoints[i].size();
		myNorm_ /= n; // cosi ho la distanza media 
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		myPerViewErrors[i] = (float)myNorm_;
		totalErr += err*err;
		totalPoints += n;

		//  visualizza vettori discrepanze. Dovrebbero distribuirsi come nuvole di punti gaussian2 nel 2D.
		//  Usando Image Watch  si puo individuare il frame responsabile dell'errore maggiore.
		Mat vett = Mat(imagePoints[i]) - Mat(imagePoints2);
		float rescaleError = 100.;
		for (int ii = 0; ii < mNySquares; ++ii)
			for (int jj = 0; jj < mNxSquares; ++jj) {
				Point2f pt2 = cv::Point(100 + jj * 60, 100 + ii * 60);
				Point2f pt1 = pt2 + rescaleError * vett.at<Point2f>(ii*mNySquares + jj);
				int col = min(200 + i, 255);
				line(figura, pt1, pt2, cv::Scalar(col,col,col), 1, 8, 0);
			}
	}
	imshow("Reproj. errors distr.", figura);
	waitKey(0);

	// Filtra le viste in base al reprojection error.
	// Se l'errore di una vista è maggiore di  rejectionFactor * mediana(errori vista)  la vista viene eliminata
	float errMediano = mediana(perViewErrors, 0.5); // 0.5 per avere la mediana
	imagePointsFiltered.clear();
	//for (i = 0; i < (int)objectPoints.size(); ++i)
	int elementiSaltati = 0;
	for (i = 0; i < (int)listFiles->size(); ++i){
		if ((*listFiles)[i] == "NotFound")
			elementiSaltati++;

		if (i < imagePoints.size()){
			if (perViewErrors[i] < (errMediano * rejectFactor))
				imagePointsFiltered.push_back(imagePoints[i]);
			else {
				cout << (*listFiles)[i + elementiSaltati] << " Deleted" << endl;
				(*listFiles)[i + elementiSaltati] = string("Deleted"); // elimina dalla lista delle immagini quella con errore troppo alto
			}
		}
	}

	return std::sqrt(totalErr / totalPoints);
}


int Calibration::SaveResults() {

	FileStorage fs(mNameOut, FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm t2;
	localtime_s(&t2, &tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", &t2);

	fs << "calibration_Time" << buf;

	if (!rvecs.empty())
		fs << "nrOfFrames" << (int)rvecs.size();
		fs << "image_Width" << imageSize.width;
		fs << "image_Height" << imageSize.height;
		fs << "board_Width" << mNxSquares;
		fs << "board_Height" << mNySquares;
		fs << "square_Size" << mDimSquare;

	if (flag & cv::CALIB_FIX_ASPECT_RATIO)
		fs << "FixAspectRatio" << mAspectRatio;

	if (flag)
	{
		sprintf_s(buf, "flags: %s%s%s%s",
			flag & cv::CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
			flag & cv::CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
			flag & cv::CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
			flag & cv::CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "");
//		cvWriteComment(*fs, buf, 0);

	}

	fs << "flagValue" << flag;

	fs << "Camera_Matrix" << cameraMatrix;
	fs << "Distortion_Coefficients" << distCoeffs;

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
//		cv::FileStorage::writeComment("a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "Extrinsic_Parameters" << bigmat;
	}

	//if (!imagePoints.empty())
	//{
	//	Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
	//	for (int i = 0; i < (int)imagePoints.size(); i++)
	//	{
	//		Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
	//		Mat imgpti(imagePoints[i]);
	//		imgpti.copyTo(r);
	//	}
	//	fs << "Image_points" << imagePtMat;
	//}

	return 1;
};



// type ha il seguente formato  "%s*.*"  per tutto il contenuto,  "%s*.jpg"  per i file jpg e cosi via
// folder  è il path es :  "G:\\Nader\\1\\"
vector<string> Calibration::get_all_files_names_within_folder(string type,string folder)
{
	vector<string> names;
	char search_path[200];
	sprintf_s(search_path, type.c_str(), folder.c_str());
	WIN32_FIND_DATA fd;
	HANDLE hFind = ::FindFirstFile(search_path, &fd);
	if (hFind != INVALID_HANDLE_VALUE) {
		do {
			// read all (real) files in current folder
			// , delete '!' read other 2 default folder . and ..
			if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
				names.push_back(fd.cFileName);
			}
		} while (::FindNextFile(hFind, &fd));
		::FindClose(hFind);
	}
	return names;
}