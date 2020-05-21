#include <Camera.h>

void camera::imwrite() {//todo gray2color
	cv::Mat colorimage = cv::Mat(cv::Size(viewWidth_, viewHeight_), CV_32FC1), us8img;
	memcpy(colorimage.data, getReflectanceData(), sizeof(float) * colorimage.size().width*colorimage.size().height);
	colorimage.convertTo(us8img, CV_8UC1, 256.0f);
	cv::imwrite(fileBase+std::to_string(count) +".jpg", us8img);	
	count++;
}