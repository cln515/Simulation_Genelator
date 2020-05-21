#include "PanoramaRenderer\PanoramaRenderer.h"
#include "opencv2/opencv.hpp"

class camera:public PanoramaRenderer{
public:
	double fps = 20.0;
	std::string fileBase;
	int count = 0;

	void imwrite();
};

