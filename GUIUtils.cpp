
#include "GUIUtils.h"

using namespace std;
using namespace cv;

int alpha_slider_max = 100;
int alpha_slider = 0;
int BORDER = 8;  // Border between GUI elements to the edge of the image.
Rect m_BtnNormal;
Rect m_BtnFast;
int FastPlot = 0;
int sThreshold = 0; // sensiblity


// Scrive un testo dentro all'immagine. top-left giustificato,  ma con valori negativi di x e y  si puo giustificare al centro.
// restituisce il bounding box del rettangolo attorno al testo
Rect drawString(Mat img, string text, Point coord, Scalar color, float fontScale , int thickness , int fontFace )
{
	int baseline = 0;
	Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
	baseline += thickness;

	// Adjust the coords for left/right-justified or top/bottom-justified.
	if (coord.y >= 0) {
		// Coordinates are for the top-left corner of the text from the top-left of the image, so move down by one row.
		coord.y += textSize.height;
	}
	else {
		// Coordinates are for the bottom-left corner of the text from the bottom-left of the image, so come up from the bottom.
		coord.y += img.rows - baseline + 1;
	}
	// Become right-justified if desired.
	if (coord.x < 0) {
		coord.x += img.cols - textSize.width + 1;
	}

	// Get the bounding box around the text.
	Rect boundingRect = Rect(coord.x, coord.y - textSize.height, textSize.width, baseline + textSize.height);

	// Draw anti-aliased text.
	putText(img, text, coord, fontFace, fontScale, color, thickness, cv::LINE_AA);

	// Let the user know how big their text is, in case they want to arrange things.
	return boundingRect;
}

// Disegna il GUI button nell'immagine usando drawString().
// Si puo specificare un minWidth se si vogliono ottenere vari buttons con la stessa width.
// Restituisce il bounding box rect attorno al bottone disegnato, cosi si possono posizionare buttons vicini (vedere l'uso).
Rect drawButton(Mat img, string text, Point coord, int minWidth)
{
	int B = BORDER;
	Point textCoord = Point(coord.x + B, coord.y + B);
	Rect rcText = drawString(img, text, textCoord, CV_RGB(0, 0, 0));
	// Draw a filled rectangle around the text.
	Rect rcButton = Rect(rcText.x - B, rcText.y - B, rcText.width + 2 * B, rcText.height + 2 * B);
	// Set a minimum button width.
	if (rcButton.width < minWidth)
		rcButton.width = minWidth;
	// Make a semi-transparent white rectangle.
	Mat matButton = img(rcButton);
	matButton += CV_RGB(90, 90, 90);
	// Draw a non-transparent white border.
	rectangle(img, rcButton, CV_RGB(200, 200, 200), 1, cv::LINE_AA);

	// Draw the actual text that will be displayed, using anti-aliasing.
	drawString(img, text, textCoord, CV_RGB(10, 255, 20));
	//drawString(img, text, textCoord, CV_RGB(10, 55, 20));

	return rcButton;
}

// Verifica se il punto è dentro una data regione
bool isPointInRect(const Point pt, const Rect rc)
{
	if (pt.x >= rc.x && pt.x <= (rc.x + rc.width - 1))
		if (pt.y >= rc.y && pt.y <= (rc.y + rc.height - 1))
			return true;

	return false;
}

// Mouse event handler. 
// chiamato automaticamente da OpenCV quando l'utente clicca nel GUI window.
void onMouse(int event, int x, int y, int, void*)
{
	// We only care about left-mouse clicks, not right-mouse clicks or mouse movement.
	if (event != cv::EVENT_LBUTTONDOWN)
		return;

	// Check if the user clicked on one of our GUI buttons.
	Point pt = Point(x, y);
	if (isPointInRect(pt, m_BtnFast)) {
		FastPlot = 1;
	}
	if (isPointInRect(pt, m_BtnNormal)) {
		FastPlot = 0;
	}

}

// I range delle trackbar per default vanno da 0 a 100
void on_trackbarSensibility(int, void*)
{
	float alpha = (double)alpha_slider / alpha_slider_max;
	//int iThreshold = 20; // DOG intensity threshold tipico
	sThreshold = (int)(alpha*100.); // cioe da 0 a 100
	if (sThreshold < 10) sThreshold = 10; // ma con un minimo a 10
}

// esempio addTimeDateToString("video", ".avi")
string addTimeDateToString(string &prefix, string &suffix)
{
	time_t t = time(0);   // get time now
	struct tm now;
	bool LocTime = true;
	try {
		localtime_s(&now, &t);
	}
	catch (...)
	{
		LocTime = false;
	}

	ostringstream oss;
	std::string var = oss.str();

	oss << prefix << (now.tm_year + 1900) << '-'
		<< (now.tm_mon + 1) << '-'
		<< now.tm_mday << '_'
		<< now.tm_hour << '-'
		<< now.tm_min << '-'
		<< now.tm_sec << suffix;
	return oss.str();
}

int ReadAvi(string filename) {

	VideoCapture cap(filename.c_str());
	if (!cap.isOpened())
		return -1;

	namedWindow("video", 1);
	for (;;)
	{
		Mat frame;
		try {
			cap >> frame; // get a new frame from camera
			imshow("video", frame);
			if (waitKey(1) >= 0) break;
		}
		catch (...){
			// fine del file
			return 0;
		}
	}

	return 0;
}

