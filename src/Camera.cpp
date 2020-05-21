#include <Camera.h>

void camera::imwrite() {
	cv::Mat colorimage = cv::Mat(cv::Size(viewWidth_, viewHeight_), CV_8UC3);
	memcpy(colorimage.data, getColorData(), sizeof(uchar)* 3 * colorimage.size().width*colorimage.size().height);
	cv::imwrite(fileBase+std::to_string(count) +".jpg", colorimage);
	count++;
}