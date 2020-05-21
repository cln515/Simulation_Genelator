#include <LRF_emu.h>
#include <Camera.h>
#include <spline.h>


class FusionPlatform {
public:
	std::vector<LRF::LRF_emulator*> lidars;
	std::vector<Matrix4d> ext_Lidar;
	std::vector<camera*> cameras;
	std::vector<Matrix4d> ext_camera;

	void scan(double ts, double te);

	void setTime(std::vector<double> time,std::vector<Matrix4d> motion);

private:
	tk::spline sx,sy,sz,sqx,sqy,sqz,sqw;
};