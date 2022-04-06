#ifndef STEREOIMAGEEVENTHANDLER_H
#define STEREOIMAGEEVENTHANDLER_H
#define WIN64

#include <pylon/PylonIncludes.h>
#include <GenApi/INodeMap.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/_BaslerUniversalCameraParams.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <pylon/TlFactory.h>
#include <iostream>

using namespace Pylon;
using namespace cv;

typedef struct CameraImage {
	cv::Mat view0;
	cv::Mat view1;
	uint64_t frameNumberL;
	uint64_t frameNumberR;
	//bool both_taken;
};

class StereoImageEventHandler : public CImageEventHandler
{
public:
	StereoImageEventHandler();
	~StereoImageEventHandler() {};
	void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult) override;
	CameraImage SendData();
	int glFrame = 0; // global frame number (increased ad each GetFrames() call)
	float FactorResize = .5;
	CPylonImage pylonImage;
	CameraImage StereoImage;
	CImageFormatConverter formatConverter;
	int64_t countr;
	int64_t countl;
};
#endif