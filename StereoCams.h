#ifndef STEREOCAMS_H
#define STEREOCAMS_H
#define WIN64

#include <pylon/PylonIncludes.h>
#include <GenApi/INodeMap.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <opencv2/core/core.hpp>
#include <pylon/TlFactory.h>
#include "StereoImageEventHandler.h"

using namespace Pylon;

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
	//void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult) override;
	cv::Mat view0;
	cv::Mat view1;
	int64_t frame0;
	int64_t frame1;
	int glFrame = 0; // global frame number (increased ad each GetFrames() call)
	float FactorResize = .5;
	static int frames;
	bool EnableView = 1;
private:
	Pylon::CBaslerUniversalInstantCameraArray pcameras;
	//StereoImageEventHandler myEventH;
	unsigned int numCameras;
	void PrintBuildInfo(void);
	//Pylon::CGrabResultPtr** ptrGrabResult;
	double globalTime = 0.;
};
#endif