#define WIN64

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class Calibration
{
public:
	Calibration() : mDimSquare(4.0), mNxSquares(8), mNySquares(6), mNameOut("CalibParams.yml") {};
	Calibration(float dimSquare, int nxSquares, int nySquares, std::string NameOut) :
		mDimSquare(dimSquare), mNxSquares(nxSquares), mNySquares(nySquares), mNameOut(NameOut) {};
	~Calibration() ;

	void readSettings(const cv::FileNode& node);
	void writeSettings(cv::FileStorage& fs) const;
	void SetViewOn(bool v) { mViewOn = v; };
	int SetImages(std::vector<std::string> &vectImages); // vector of filenames
	int SetImages(std::string pathName, std::string estension); // the entire content of this path
	int Calibrate(float rejectionFactor, bool objpntflag);
	int ObjectPointsEstimation(float rejectionFactor);
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	std::vector<cv::Point3f> checkboardCorners;
	std::vector<std::string> *listFiles;
private:
	cv::Size imageSize;
	bool mViewOn = true;
	float mDimSquare = 0.;
	int mNxSquares =0;
	int mNySquares =0;
	float mAspectRatio = 1.0;
	bool mFlipVertical = false;
	bool mCalibZeroTangentDist = false;
	bool mCalibFixPrincipalPoint = false;
	void interprate();
	bool goodInput;
	int flag; // flags per la modalità di calibrazione
	int SaveResults();
	std::string mNameOut;
	std::vector<std::string> get_all_files_names_within_folder(std::string type,std::string folder);
	std::string mPath;
	double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints, const std::vector<std::vector<cv::Point2f>> &imagePoints, std::vector<float> &perViewErrors, float rejectFactor, std::vector<std::vector<cv::Point2f>> &imagePointsFiltered);

};
