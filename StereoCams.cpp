#include <time.h>
#include <ostream>
#include <string>
#include "StereoCams.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define TIMESTAMP

using namespace Pylon;
using namespace GenApi;
using namespace Basler_UniversalCameraParams;
using namespace cv;
using namespace std;


//
// Funzioni  globali della telecamera
//
/*
void PrintCameraInfo(FlyCapture2::CameraInfo* pCamInfo)
{
	printf(
		"\n*** CAMERA INFORMATION ***\n"
		"Serial number - %u\n"
		"Camera model - %s\n"
		"Camera vendor - %s\n"
		"Sensor - %s\n"
		"Resolution - %s\n"
		"Firmware version - %s\n"
		"Firmware build time - %s\n\n",
		pCamInfo->serialNumber,
		pCamInfo->modelName,
		pCamInfo->vendorName,
		pCamInfo->sensorInfo,
		pCamInfo->sensorResolution,
		pCamInfo->firmwareVersion,
		pCamInfo->firmwareBuildTime);
}

void PrintError(FlyCapture2::Error error)
{
	error.PrintErrorTrace();
};


bool CheckSoftwareTriggerPresence(Camera* pCam)
{
	const unsigned int k_triggerInq = 0x530;

	FlyCapture2::Error error;
	unsigned int regVal = 0;

	error = pCam->ReadRegister(k_triggerInq, &regVal);

	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return false;
	}

	if ((regVal & 0x10000) != 0x10000)
	{
		return false;
	}

	return true;
}

bool PollForTriggerReady(Camera* pCam)
{
	const unsigned int k_softwareTrigger = 0x62C;
	FlyCapture2::Error error;
	unsigned int regVal = 0;

	do
	{
		error = pCam->ReadRegister(k_softwareTrigger, &regVal);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return false;
		}

	} while ((regVal >> 31) != 0);

	return true;
}

bool FireSoftwareTrigger(Camera* pCam)
{
	const unsigned int k_softwareTrigger = 0x62C;
	const unsigned int k_fireVal = 0x80000000;
	FlyCapture2::Error error;

	error = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return false;
	}

	return true;
}

int SetTriggerModeHD(bool status, Camera &camera){

	FlyCapture2::Error error;

	// Get current trigger settings
	TriggerMode triggerMode;
	error = camera.GetTriggerMode(&triggerMode);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}

	// Set camera to trigger mode 0
	triggerMode.onOff = status;
	triggerMode.mode = 0;
	triggerMode.polarity = 1;
	triggerMode.parameter = 0;

	if (status == true) {
		// Triggering the camera externally using source 0.
		triggerMode.source = 0;
	}
	else
	{
		// A source of 7 means software trigger
		triggerMode.source = 7;
	}

	error = camera.SetTriggerMode(&triggerMode);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}



	// attiva il timestamp
	EmbeddedImageInfo EInfo;
	camera.GetEmbeddedImageInfo(&EInfo);
	EInfo.timestamp.onOff = true;
	error = camera.SetEmbeddedImageInfo(&EInfo);

	//// Poll to ensure camera is ready
	//bool retVal = PollForTriggerReady(&camera);
	//if (!retVal)
	//{
	//	printf("\nError polling for trigger ready!\n");
	//	return -1;
	//}

	return 1;
}

*/

StereoCams::StereoCams(){

	//FlyCapture2::Error error;

	CamerasReady = false;
	//PrintBuildInfo();

	/*BusManager busMgr;
	error = busMgr.GetNumOfCameras(&numCameras);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
	}
	*/
	//PylonInitialize();
	//PylonAutoInitTerm();

	// Get the transport layer factory.

	CTlFactory& tlFactory = CTlFactory::GetInstance();

	// Get all attached devices and exit application if no device is found.
	DeviceInfoList_t devices;
	if (tlFactory.EnumerateDevices(devices) == 0)
	{
		throw RUNTIME_EXCEPTION("No camera present.");
	}

	if (tlFactory.EnumerateDevices(devices) == 1)
	{
		throw RUNTIME_EXCEPTION("Only one camera presents.");
	}
	pcameras.Initialize(2);
	// Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
	ECleanup  cleanupProcedure = Cleanup_Delete;

	if (pcameras.GetSize() > 0) {
		if (pcameras.IsGrabbing())
			pcameras.StopGrabbing();
		pcameras.Close();
		pcameras.DetachDevice();
		pcameras.DestroyDevice();
	}

	// Create and attach all Pylon Devices.
	for (size_t i = 0; i < pcameras.GetSize(); ++i)
	{
		pcameras[i].Attach(tlFactory.CreateDevice(devices[i]));

		if (pcameras[i].GetSfncVersion() >= Sfnc_2_0_0)
		{
			// Print the model name of the camera.
			cout << "Using device " << pcameras[i].GetDeviceInfo().GetModelName() << endl;
		}

	}

	
	//pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new CImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
	//pcameras[1].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new CImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
	// pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new StereoImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
	//pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler((Pylon::CImageEventHandler*) &(myEventH), RegistrationMode_Append, Cleanup_Delete);
	//pcameras[1].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new StereoImageEventHandler, RegistrationMode_Append, Cleanup_Delete);

	// Set the upper limit of the camera's frame rate to 30 fps
	pcameras.Open();
	//pcameras[1].Open();

	//pcameras[0]->AcquisitionFrameRateEnable.SetValue(TRUE);
	//pcameras[1]->AcquisitionFrameRateEnable.SetValue(TRUE);
	//pcameras[0]->AcquisitionFrameRate.SetValue(50.0);
	//pcameras[1]->AcquisitionFrameRate.SetValue(50.0);

	pcameras[0].TriggerSelector.SetValue(TriggerSelector_FrameStart);
	pcameras[0].TriggerMode.SetValue(TriggerMode_On);
	pcameras[0].TriggerSource.SetValue(TriggerSource_Line3);
	pcameras[0].ChunkModeActive.SetValue(true);
	pcameras[0].ChunkSelector.SetValue(ChunkSelector_Timestamp);
	pcameras[0].ChunkEnable.SetValue(true);
	pcameras[0].BslChunkTimestampSelector.SetValue(BslChunkTimestampSelector_ExposureStart);


	pcameras[1].TriggerSelector.SetValue(TriggerSelector_FrameStart);
	pcameras[1].TriggerMode.SetValue(TriggerMode_On);
	pcameras[1].TriggerSource.SetValue(TriggerSource_Line3);
	pcameras[1].ChunkModeActive.SetValue(true);
	pcameras[1].ChunkSelector.SetValue(ChunkSelector_Timestamp);
	pcameras[1].ChunkEnable.SetValue(true);
	pcameras[1].BslChunkTimestampSelector.SetValue(BslChunkTimestampSelector_ExposureStart);

	pcameras[0].ExposureTime.SetValue(1900.0);
	pcameras[1].ExposureTime.SetValue(1900.0);

	pcameras[0].TimestampLatch.Execute();
	pcameras[1].TimestampLatch.Execute();

	startingTimeCamera0 = pcameras[0].TimestampLatchValue.GetValue();
	startingTimeCamera1 = pcameras[1].TimestampLatchValue.GetValue();

	/*
	printf("Number of cameras detected: %u\n", numCameras);

	if (numCameras < 1)
	{
		printf("Need al least 2 cameras....\n");
	}

	ppCameras = new Camera*[numCameras];
	
	// Connect to all detected cameras and attempt to set them to
	// a common video mode and frame rate
	for (unsigned int i = 0; i < numCameras; i++)
	{
		ppCameras[i] = new Camera();

		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(i, &guid);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
		}

		// Connect to a camera
		error = ppCameras[i]->Connect(&guid);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
		}

		// Get the camera information
		CameraInfo camInfo;
		error = ppCameras[i]->GetCameraInfo(&camInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
		}

		// set GPIO
		TriggerMode mTrigger;
		mTrigger.mode = 0;
		mTrigger.source = 0;
		mTrigger.parameter = 0;
		mTrigger.onOff = true;
		mTrigger.polarity = 1;
		ppCameras[i]->SetTriggerMode(&mTrigger);

		TriggerMode mVerifica;
		ppCameras[i]->GetTriggerMode(&mVerifica);
		int breakpoipnt = 0;

	}

	// ordina le camere per n. seriale
	CameraInfo camInfo0;
	CameraInfo camInfo1;
	error = ppCameras[0]->GetCameraInfo(&camInfo0);
	error = ppCameras[1]->GetCameraInfo(&camInfo1);
	if (camInfo0.serialNumber > camInfo1.serialNumber) {
		// swap
		Camera *pC = ppCameras[0];
		ppCameras[0] = ppCameras[1];
		ppCameras[1] = pC;
	}
	*/
	CamerasReady = true;
};

//constructor for calibration
StereoCams::StereoCams(char calib_flag) {

	if (calib_flag == 'c')
	{
		//FlyCapture2::Error error;

		CamerasReady = false;
		//PrintBuildInfo();

		/*BusManager busMgr;
		error = busMgr.GetNumOfCameras(&numCameras);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
		}
		*/
		//PylonInitialize();
		//PylonAutoInitTerm();

		// Get the transport layer factory.

		CTlFactory& tlFactory = CTlFactory::GetInstance();

		// Get all attached devices and exit application if no device is found.
		DeviceInfoList_t devices;
		if (tlFactory.EnumerateDevices(devices) == 0)
		{
			throw RUNTIME_EXCEPTION("No camera present.");
		}

		if (tlFactory.EnumerateDevices(devices) == 1)
		{
			throw RUNTIME_EXCEPTION("Only one camera presents.");
		}
		pcameras.Initialize(2);
		// Create an array of instant cameras for the found devices and avoid exceeding a maximum number of devices.
		ECleanup  cleanupProcedure = Cleanup_Delete;

		if (pcameras.GetSize() > 0) {
			if (pcameras.IsGrabbing())
				pcameras.StopGrabbing();
			pcameras.Close();
			pcameras.DetachDevice();
			pcameras.DestroyDevice();
		}

		// Create and attach all Pylon Devices.
		for (size_t i = 0; i < pcameras.GetSize(); ++i)
		{
			pcameras[i].Attach(tlFactory.CreateDevice(devices[i]));

			if (pcameras[i].GetSfncVersion() >= Sfnc_2_0_0)
			{
				// Print the model name of the camera.
				cout << "Using device " << pcameras[i].GetDeviceInfo().GetModelName() << endl;
			}

		}


		//pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new CImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		//pcameras[1].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new CImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		// pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new StereoImageEventHandler, RegistrationMode_Append, Cleanup_Delete);
		//pcameras[0].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler((Pylon::CImageEventHandler*) &(myEventH), RegistrationMode_Append, Cleanup_Delete);
		//pcameras[1].Pylon::CBaslerUniversalInstantCamera::RegisterImageEventHandler(new StereoImageEventHandler, RegistrationMode_Append, Cleanup_Delete);

		// Set the upper limit of the camera's frame rate to 30 fps
		pcameras.Open();
		//pcameras[1].Open();

		//pcameras[0]->AcquisitionFrameRateEnable.SetValue(TRUE);
		//pcameras[1]->AcquisitionFrameRateEnable.SetValue(TRUE);
		//pcameras[0]->AcquisitionFrameRate.SetValue(50.0);
		//pcameras[1]->AcquisitionFrameRate.SetValue(50.0);

		pcameras[0].TriggerSelector.SetValue(TriggerSelector_FrameStart);
		pcameras[0].TriggerMode.SetValue(TriggerMode_On);
		pcameras[0].TriggerSource.SetValue(TriggerSource_Line3);
		pcameras[0].ChunkModeActive.SetValue(true);
		pcameras[0].ChunkSelector.SetValue(ChunkSelector_Timestamp);
		pcameras[0].ChunkEnable.SetValue(true);
		pcameras[0].BslChunkTimestampSelector.SetValue(BslChunkTimestampSelector_ExposureStart);


		pcameras[1].TriggerSelector.SetValue(TriggerSelector_FrameStart);
		pcameras[1].TriggerMode.SetValue(TriggerMode_On);
		pcameras[1].TriggerSource.SetValue(TriggerSource_Line3);
		pcameras[1].ChunkModeActive.SetValue(true);
		pcameras[1].ChunkSelector.SetValue(ChunkSelector_Timestamp);
		pcameras[1].ChunkEnable.SetValue(true);
		pcameras[1].BslChunkTimestampSelector.SetValue(BslChunkTimestampSelector_ExposureStart);

		pcameras[0].ExposureTime.SetValue(380000.0);
		pcameras[1].ExposureTime.SetValue(380000.0);

		pcameras[0].TimestampLatch.Execute();
		pcameras[1].TimestampLatch.Execute();

		startingTimeCamera0 = pcameras[0].TimestampLatchValue.GetValue();
		startingTimeCamera1 = pcameras[1].TimestampLatchValue.GetValue();


		CamerasReady = true;
	}

};

void StereoCams::PrintBuildInfo(void)
{
	//FlyCapture2::FC2Version fc2Version;
	//Utilities::GetLibraryVersion(&fc2Version);
	//char version[128];
	/*
	sprintf_s(
		version,
		"FlyCapture2 library version: %d.%d.%d.%d\n",
		fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build);

	printf("%s", version);
	*/
	char timeStamp[512];
	//sprintf_s(timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__);

	printf("%s", timeStamp);
};


int StereoCams::StartCaptureCameras(){

	//FlyCapture2::Error error;
	// try catch inutile
		/*
		error = ppCameras[0]->StartCapture();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			printf("Error starting camera 0. \n Press Enter to exit. \n");
			getchar();
			return -1;
		}
		error = ppCameras[1]->StartCapture();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			printf("Error starting cameras 1. \n Press Enter to exit. \n");
			getchar();
			return -1;
		}
		*/
	// Starts grabbing for all cameras starting with index 0. The grabbing
	// is started for one camera after the other. That's why the images of all
	// cameras are not taken at the same time.
	// However, a hardware trigger setup can be used to cause all cameras to grab images synchronously.
	// According to their default configuration, the cameras are
	// set up for free-running continuous acquisition.
	// This smart pointer will receive the grab result data.
	//pcameras[0].MaxNumBuffer = 16;
	//pcameras[1].MaxNumBuffer = 16;


	
	//pcameras.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
	//pcameras[1].StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
	if (pcameras[0].CanWaitForFrameTriggerReady() && pcameras[1].CanWaitForFrameTriggerReady())
	pcameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
	//pcameras[1].StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);

	return 1;
};

int StereoCams::StopCaptureCameras(){

	for (unsigned int i = 0; i < 2; i++)
	{
		//cv::waitKey(500);
		pcameras[i].StopGrabbing();
		pcameras[i].DetachDevice();
		pcameras[i].DestroyDevice();
		//delete pcameras[i];
		//PylonTerminate();
		cv::waitKey(500);
	}


	printf("Cameras stopped! Press Enter to continue...\n");
	getchar();

	return 1;
};


void StereoCams::SetViewOn(){
	EnableView = true;
};
void StereoCams::SetViewOff(){
	EnableView = false;
};


// press  s  to get the snapshots
// press  q to quit the procedure
int StereoCams::GetSelectedFrames(float FactorResize) {

	char key = 0; 	int j = 0;
	clock_t start1 = clock();
	double msec;
	while ((key != 'q'))
	{
		j++;
		key = GetFrames(FactorResize, msec);

		clock_t start2 = clock();
		double elapsed_secs = double(start2 - start1) / CLOCKS_PER_SEC;
		float actualFrRate = j / elapsed_secs;
		start1 = start2;
		//printf("Frame rate  %f \n", actualFrRate);
		//cout << "Frame rate  %f \n" << actualFrRate << endl;

		// snapshot 
		if (key == 's') {
			return 1;
		}
	}


	return -1;
}

//  restituisce -1 se nessun tasto è stato premuto
//  msec sono i millisecondi trascorsi dal frame precedente
char StereoCams::GetFrames(float FactorResize, double& msec) {

	char key;
	//int both_taken = 0;
	//FlyCapture2::Error error;
	glFrame++;

	// ######################################################### OLD WAY ################################################################
	//TimeStamp timestamp[2]; 
	CImageFormatConverter formatConverter;
	formatConverter.OutputPixelFormat = PixelType_BGR8packed;
	CPylonImage pylonImage;
	CPylonImage pylonImage1;
	CGrabResultPtr ptrGrabResult;
	CGrabResultPtr ptrGrabResult1;
	int diff_frame = 0;
	pcameras[0].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
	pcameras[1].RetrieveResult(5000, ptrGrabResult1, TimeoutHandling_ThrowException);
	//frame0 = ptrGrabResult->GetID();
	//frame1 = ptrGrabResult1->GetID();
	/*timeStamp0 = ptrGrabResult->GetTimeStamp() - startingTimeCamera0;
	timeStamp1 = ptrGrabResult1->GetTimeStamp() - startingTimeCamera1;*/
	diff_timeStamp = (double)((double)(ptrGrabResult->GetTimeStamp() - startingTimeCamera0) - (double)(ptrGrabResult1->GetTimeStamp() - startingTimeCamera1)) / 1000000.0;
	if (ptrGrabResult->GrabSucceeded() && ptrGrabResult1->GrabSucceeded())
	{
		//cout << "   ts0 - ts1 : " <<  diff_timeStamp << endl;
			//diff_frame = frame0 - frame1;
			if ((diff_timeStamp <= 1) && (diff_timeStamp >= -1))
			{
				//cout << diff_frame << endl;
				formatConverter.Convert(pylonImage, ptrGrabResult);
				formatConverter.Convert(pylonImage1, ptrGrabResult1);
				Mat oldView = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
				Mat oldView1 = Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t*)pylonImage1.GetBuffer());
				//intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
				//int64_t frameNumber = ptrGrabResult->GetID();
				resize(oldView, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
				resize(oldView1, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
				//cout << "FrameCounter_l (Result): " << ptrGrabResult->GetID() << "FrameCounter_r (Result): " << ptrGrabResult1->GetID() << endl;
				//cout << "FrameCounter_rc (Result): " << ptrGrabResult1->GetImageNumber() << endl;
				//pcameras[1].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
				//formatConverter.Convert(pylonImage, ptrGrabResult);
				////formatConverter.Convert(pylonImage1, ptrGrabResult1);
				//oldView = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
				//resize(oldView, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
				//formatConverter.Convert(pylonImage, ptrGrabResult);
				//formatConverter.Convert(pylonImage1, ptrGrabResult1);
				//oldView = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
				//Mat oldView1 = Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t*)pylonImage1.GetBuffer());
				//intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
				//if ( frameNumber != ptrGrabResult->GetID()) cout << "L: " << frameNumber << "R: " << ptrGrabResult->GetID() << endl;
				//if (cameraContextValue == 0)
				//{
				//	resize(oldView, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
				//	both_taken += 1;
				//	cout << "L: " << frameNumber << endl;
				//}
				//if (cameraContextValue == 1)
				//{
				//	resize(oldView, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
				//	both_taken = 2;
				//	cout << "R: " << frameNumber << endl;
				//}
				//resize(oldView, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
			//}
				//resize(oldView, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
				putText(view0, "L", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
				//putText(view0, to_string(frame0), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
				if (EnableView)
					imshow("Left", view0);
				putText(view1, "R", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
				//putText(view1, to_string(frame1), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
				if (EnableView)
					imshow("Right", view1);

				pcameras[0].TimestampLatch.Execute();
				pcameras[1].TimestampLatch.Execute();

				startingTimeCamera0 = pcameras[0].TimestampLatchValue.GetValue();
				startingTimeCamera1 = pcameras[1].TimestampLatchValue.GetValue();
			}
			else
			{
				//pcameras.StopGrabbing();
				//if (pcameras[0].CanWaitForFrameTriggerReady() && pcameras[1].CanWaitForFrameTriggerReady())
				//	pcameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
				pcameras[1].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
				pcameras[0].RetrieveResult(5000, ptrGrabResult1, TimeoutHandling_ThrowException);
				cout << diff_timeStamp << endl;
			}
				
			//else if (diff_timeStamp != 0)
			//{
			//	//cout << "R : " << ptrGrabResult1->GetID() << endl;
			//	pcameras[1].RetrieveResult(5000, ptrGrabResult1, TimeoutHandling_ThrowException);
			//	pcameras[0].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
			//	frame0 = ptrGrabResult->GetID();
			//	frame1 = ptrGrabResult1->GetID();
			//	timeStamp0 = ptrGrabResult->GetTimeStamp() - startingTimeCamera0;
			//	timeStamp1 = ptrGrabResult1->GetTimeStamp() - startingTimeCamera1;
			//	diff_timeStamp = (timeStamp0 - timeStamp1) / 1000000;
			//	cout << "s_frame0 : " << frame0 << "   s_frame1 : " << frame1 << "   s_ts0 - s_ts1 : " << diff_timeStamp << endl;
			//	//cout << "diff_frame =  " << diff_frame << endl;
			//	/*diff_frame = frame0 - frame1;
			//	cout << "diff_frame: " << diff_frame << endl;*/
			//	formatConverter.Convert(pylonImage, ptrGrabResult);
			//	formatConverter.Convert(pylonImage1, ptrGrabResult1);
			//	Mat oldView = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
			//	Mat oldView1 = Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t*)pylonImage1.GetBuffer());
			//	//intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
			//	//int64_t frameNumber = ptrGrabResult->GetID();
			//	resize(oldView, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
			//	resize(oldView1, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
			//	putText(view0, "L", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	putText(view0, to_string(frame0), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	if (EnableView)
			//		imshow("Left", view0);
			//	putText(view1, to_string(frame1), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	putText(view1, "R", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	if (EnableView)
			//		imshow("Right", view1);
			//}				
			//else if (diff_frame == -1)
			//{
			//	cout << "L : " << ptrGrabResult->GetID() << endl;
			//	pcameras[0].RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);
			//	frame0 = ptrGrabResult->GetID();
			//	timeStamp0 = ptrGrabResult->GetTimeStamp();
			//	cout << "diff_frame = -1   " << "frame0 : " << frame0 << "   frame1 : " << frame1 << "   ts0-ts1 : " << (timeStamp1 - timeStamp0) / 1000000 << endl;
			//	//frame1 = ptrGrabResult1->GetID();
			//	/*diff_frame = ptrGrabResult->GetID() - ptrGrabResult1->GetID();
			//	cout << "diff_frame: " << diff_frame << endl;*/
			//	formatConverter.Convert(pylonImage, ptrGrabResult);
			//	formatConverter.Convert(pylonImage1, ptrGrabResult1);
			//	Mat oldView = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
			//	Mat oldView1 = Mat(ptrGrabResult1->GetHeight(), ptrGrabResult1->GetWidth(), CV_8UC3, (uint8_t*)pylonImage1.GetBuffer());
			//	//intptr_t cameraContextValue = ptrGrabResult->GetCameraContext();
			//	//int64_t frameNumber = ptrGrabResult->GetID();
			//	resize(oldView, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
			//	resize(oldView1, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
			//	putText(view0, "L", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	putText(view0, to_string(frame0), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	if (EnableView)
			//		imshow("Left", view0);
			//	putText(view1, "R", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	putText(view1, to_string(frame1), cv::Point(40, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			//	if (EnableView)
			//		imshow("Right", view1);
			//}
			//cout << "frame_number: " << ptrGrabResult->GetID() << "      L - R : " << diff_frame << endl;
	}
	
	/*if ((StereoImage.frameNumberL == StereoImage.frameNumberR) && (StereoImage.frameNumberL!=0))
	{
		resize(StereoImage.view0, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
		putText(view0, "L", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
		putText(view0, to_string(countl), cv::Point(20, 40), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
		resize(StereoImage.view1, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
		putText(view1, "R", cv::Point(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
		putText(view1, to_string(countr), cv::Point(20, 40), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
		if (EnableView)
		{
			imshow("Right", view1);
			imshow("Left", view0);
		}
			
	}*/

	/*
	// leggo dal buffer subito le due immagini, (forse) riduco il lag nel timestamp
	Image rawImage[2];
	error = ppCameras[0]->RetrieveBuffer(&rawImage[0]);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}
	error = ppCameras[1]->RetrieveBuffer(&rawImage[1]);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}

	for (unsigned int i = 0; i < 2; i++)
	{
#ifdef TIMESTAMP
		//timestamp[i] = rawImage[i].GetTimeStamp(); // è il timestamp del PC, quando arriva l'immagine dal bus USB3
		//printf("Frame %d - Cam %d   TimeStamp  %d (sec)   -    %d (microsec)\n", glFrame, i,timestamp[i].seconds, timestamp[i].microSeconds);
#endif
		// convert to rgb
		Image rgbImage;
		rawImage[i].Convert(FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage);

		// convert to OpenCV Mat
		unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize() / (double)rgbImage.GetRows();
		Mat view = Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(), rowBytes);

		// copy the image into the public member of the class
		if (i == 0) {
			resize(view, view0, Size(), FactorResize, FactorResize, INTER_NEAREST);
			putText(view0, "L", cvPoint(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			if (EnableView)
				imshow("Left", view0);
		}

		if (i == 1){
			resize(view, view1, Size(), FactorResize, FactorResize, INTER_NEAREST);
			putText(view1, "R", cvPoint(20, 20), FONT_HERSHEY_COMPLEX_SMALL, 0.6, CV_RGB(0, 0, 255), 1, 8, false);
			if (EnableView)
				imshow("Right", view1);
		}

	}

#ifdef TIMESTAMP
	//printf("Diff time = %d \n", abs((long)timestamp[0].microSeconds - (long)timestamp[1].microSeconds));

	//double utime = (double)timestamp[0].seconds + (double)timestamp[0].microSeconds / 1000000.; // arbitrariamente la prima tc.
	//msec = (utime - globalTime) * 1000; // in msec
//	printf("microsec %d    sec %d    MSEC %f \n", timestamp[0].microSeconds, timestamp[0].seconds, utime);
	//globalTime = utime;
#endif
	*/

	if (EnableView)
		key = waitKey(1);


	return key;
}

