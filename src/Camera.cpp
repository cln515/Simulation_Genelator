#include <Camera.h>

std::string camera::imwrite() {
	cv::Mat colorimage = cv::Mat(cv::Size(viewWidth_, viewHeight_), CV_8UC3);
	memcpy(colorimage.data, getColorData(), sizeof(uchar) * 3 * colorimage.size().width*colorimage.size().height);
	if (getType() == FISHEYE) {
		cv::Mat mask = cv::Mat::zeros(cv::Size(viewWidth_, viewHeight_), CV_8UC3);
		cv::Mat dst = cv::Mat::zeros(cv::Size(viewWidth_, viewHeight_), CV_8UC3);
		cv::ellipse(mask, cv::RotatedRect(cv::Point2f(viewWidth_ / 2, viewHeight_ / 2),cv::Size2f(viewWidth_, viewHeight_),0),cv::Scalar(255.0,255.0,255.0),-1);
		//cv::circle(colorimage, cv::Point2f(viewWidth_ / 2, viewHeight_ / 2), viewWidth_ / 2, cv::Scalar(0.0, 0.0, 0.0),-1);
		colorimage.copyTo(dst, mask);
		colorimage = dst;
	}
	std::string jpgfile = fileBase + "_" + std::to_string(count) + ".jpg";
	cv::imwrite(jpgfile, colorimage);
	count++;
	return jpgfile;
}