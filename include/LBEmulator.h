#pragma once
#include "PanoramaRenderer/PanoramaRenderer.h"
#include "utility/utility.h"


struct LadybugData {
	double f;
	double cx, cy;
	double rotation[3];
	double translation[3];
};


class LBEmulator :public PanoramaRenderer {
public:
	void RenderPinhole(Matrix4d camPara,unsigned int cidx);
	void RenderPinhole(Matrix4d camPara);
	void meshNormCompute();

	LadybugData lbdata[6] = {
	{397.252,782.402,621.074,-2.43321,1.5659,-2.43315,0.041737,-0.001558,-4.6e-005},
	{405.332,793.743,603.401,-2.96221,1.56473,2.05686,0.01109,-0.040404,-0.000313},
	{403.656,786.306,615.578,1.13729,1.56559,-1.38336,-0.03482,-0.022925,0.000135},
	{401.232,787.29,621.597,3.01839,1.56118,-0.757331,-0.03276,0.026081,-5.2e-005},
	{409.449,794.926,627.289,-1.10128,1.56584,0.148267,0.014753,0.038806,0.000276},
	{401.873,781.494,623.52,0.003185,0.006302,-5.5e-005,0.001134,-0.000707,0.062821}
	};



private:
	float* norm;





};

