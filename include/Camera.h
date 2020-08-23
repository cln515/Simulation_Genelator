#include "opencv2/opencv.hpp"
#include "PanoramaRenderer/PanoramaRenderer.h"


class camera:public PanoramaRenderer{
public:
	double fps = 20.0;
	std::string fileBase;
	int count = 0;

	std::string imwrite();
};

