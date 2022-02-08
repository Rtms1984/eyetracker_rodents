#define WIN64

#include <pylon/PylonIncludes.h>
#include <GenApi/INodeMap.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <opencv2/core/core.hpp>

class StereoCams
{
public:
	StereoCams();
	~StereoCams() {};
	bool CamerasReady;  // probabilmente meglio gestito con un throw di eccezione 
	int StartCaptureCameras();
	int StopCaptureCameras();
	char GetFrames(float, double &);
	int GetSelectedFrames(float);
	void SetViewOn();
	void SetViewOff();
	cv::Mat view0;
	cv::Mat view1;
	int glFrame = 0; // global frame number (increased ad each GetFrames() call)
private:
	unsigned int numCameras;
	void PrintBuildInfo(void);
	Pylon::CBaslerUniversalInstantCamera** pcameras;
	//Pylon::CGrabResultPtr** ptrGrabResult;
	bool EnableView;
	double globalTime = 0.;
};
