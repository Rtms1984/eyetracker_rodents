#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "StereoImageEventHandler.h"


using namespace Pylon;
using namespace std;

StereoImageEventHandler::StereoImageEventHandler() {
	
	formatConverter.OutputPixelFormat = Pylon::PixelType_Mono8;
	//StereoImage.frameNumber = 1;
	countr = 0;
	countl = 0;
	
}

void StereoImageEventHandler::OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult)
	{
		//StereoImage.both_taken = 0;
		intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
		//int64_t frame = ptrGrabResult->GetImageNumber();

		//cout<< "Grabresult from camera" << cameraContextValue << ": frameNumber:  " << frameNumber <<std::endl;
		formatConverter.Convert(pylonImage, ptrGrabResult);
		cv::Mat view(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC1, (uint8_t*)pylonImage.GetBuffer());

		
		if (cameraContextValue == 0)
		{
			countl++;
			//cout << "L : " << countl << endl;
			StereoImage.view0 = view.clone();
			StereoImage.frameNumberL = countl;
		}
		if (cameraContextValue == 1)
		{
			countr++;
			//cout << "R : " << countr << endl;
			StereoImage.view1 = view.clone();
			StereoImage.frameNumberR = countr;
		}
		/*if (countl == countr)
		{
			StereoImage.both_taken = 1;
			StereoImage.frameNumber = countl;
		}*/
		
}

CameraImage StereoImageEventHandler::SendData()
{

}
