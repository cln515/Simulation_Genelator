#include "LRF_emu.h"

using namespace LRF;

std::vector<LRF_emulator::ScanPoint> LRF_emulator::scan(double t, Matrix4d campara) {
	std::vector<Vector3d> laserOrigin;
	std::vector<Vector3d> laserDirection;
	std::vector<LRF_emulator::ScanPoint> scanpt;
	lidar->getRayDirection(t,campara,laserOrigin,laserDirection);
	//std::cout<< laserOrigin.size() <<std::endl;
	std::random_device seed_gen;
	std::default_random_engine engine(seed_gen());

	std::normal_distribution<> dist(0.0, scanNoise);
	for (int i = 0; i < laserOrigin.size(); i++) {
		Vector3d retPoint;
		double reflectance;
		if ((reflectance = isearch->query_ray(laserOrigin.at(i), laserDirection.at(i), retPoint)) >= 0 && reflectance <= 1.0 && (reflectance == reflectance)) {
			Vector4d retPoint4d;
			retPoint4d << retPoint(0), retPoint(1), retPoint(2), 1;
			Vector4d localPoint = campara.inverse() * retPoint4d;
			LRF_emulator::ScanPoint spt;
			//noise

			double noise = dist(engine);
			Vector3d noised = noise * localPoint.segment(0, 3).normalized();
			spt.x = localPoint(0) + noised (0);
			spt.y = localPoint(1) + noised(1);
			spt.z = localPoint(2) + noised(2);
			spt.intensity=(reflectance);
			spt.ts = t;
			scanpt.push_back(spt);

		}
		else {
			LRF_emulator::ScanPoint spt;
			spt.x = 0.0;
			spt.y = 0.0;
			spt.z = 0.0;
			spt.intensity = 0.0;
			spt.ts = t;
			scanpt.push_back(spt);
		}
	}
	return scanpt;
};


void LRF_emulator::scanEmulation(Matrix4d campara,vector<float>& vert,vector<float>& intensity) {
	
	Vector3d scannerCenter = campara.block(0, 3, 3, 1);

	if (st == sensor_type::ZF_IMAGER) {
		int oneLinePoint = 2862;
		int lineNumber = 1111;
		double T = 50.0;
		double tstep = T / (lineNumber*oneLinePoint);
		for (double t = 0;t < T;t += tstep) {
			//
			//レーザーのfire direction
			Vector3d drc_zf;
			drc_zf << sin(2 * M_PI*t*lineNumber / T)*cos(M_PI*t / T), sin(2 * M_PI*t*lineNumber / T)*sin(t*M_PI / T), cos(2 * M_PI*t*lineNumber / T);//zf local		
			Vector3d drc_gl = campara.block(0,0,3,3)*drc_zf;
			//drc_gl = drc_gl - scannerCenter;

			Vector3d retPoint;
			double reflectance;
			if ((reflectance = isearch->query_ray(scannerCenter, drc_gl, retPoint)) >= 0 && reflectance<=1.0 && (reflectance==reflectance)) {
				Vector4d retPoint4d;
				retPoint4d << retPoint(0), retPoint(1), retPoint(2), 1;
				Vector4d localPoint = campara.inverse() * retPoint4d;
				vert.push_back(localPoint(0));
				vert.push_back(localPoint(1));
				vert.push_back(localPoint(2));
				intensity.push_back(reflectance);
			}
		}
		
	}
	else if (st == sensor_type::HDL_64E) {
		int oneLinePoint = 1000;
		
		double T = 0.1;
		double tstep = T / (oneLinePoint);
		for (double t = 0;t < T;t += tstep) {
			//
			//レーザーのfire direction
			double horizAngle = 2 * M_PI * t / T;
			for (int i = 0;i < 64;i++) {
				Vector3d drc_zf;

				double vertAngle =(90-((2.0 + 24.33) / 64 * i -24.33))* M_PI/180;
				drc_zf << sin(vertAngle)*cos(horizAngle), sin(vertAngle)*sin(horizAngle), cos(vertAngle);//zf local	)*sin(t*M_PI / T), cos(vertAngle), 1;//velo local				
				Vector3d drc_gl = campara.block(0, 0, 3, 3)*drc_zf;
				//drc_gl = drc_gl - scannerCenter;

				Vector3d retPoint;
				double reflectance;
				if ((reflectance = isearch->query_ray(scannerCenter, drc_gl, retPoint)) >= 0 && reflectance <= 1.0 && (reflectance == reflectance)) {
					Vector4d retPoint4d;
					retPoint4d << retPoint(0), retPoint(1), retPoint(2), 1;
					Vector4d localPoint = campara.inverse() * retPoint4d;
					vert.push_back(localPoint(0));
					vert.push_back(localPoint(1));
					vert.push_back(localPoint(2));
					intensity.push_back(reflectance);
				}
			}
		}
	}
	else if (st == sensor_type::VLP_16) {
		int oneLinePoint = 1000;

		double T = 0.1;
		double tstep = T / (oneLinePoint);
		for (double t = 0;t < T;t += tstep) {
			//
			//レーザーのfire direction
			double horizAngle = 2 * M_PI * t / T;
			for (int i = 0;i < 16;i++) {
				Vector3d drc_zf;

				double vertAngle = (90 - ((30.0) / 15 * i - 15))* M_PI / 180;
				drc_zf << sin(vertAngle)*cos(horizAngle), sin(vertAngle)*sin(horizAngle), cos(vertAngle);//zf local	)*sin(t*M_PI / T), cos(vertAngle), 1;//velo local				
				Vector3d drc_gl = campara.block(0, 0, 3, 3)*drc_zf;
				//drc_gl = drc_gl - scannerCenter;

				Vector3d retPoint;
				double reflectance;
				if ((reflectance = isearch->query_ray(scannerCenter, drc_gl, retPoint)) >= 0 && reflectance <= 1.0 && (reflectance == reflectance)) {
					Vector4d retPoint4d;
					retPoint4d << retPoint(0), retPoint(1), retPoint(2), 1;
					Vector4d localPoint = campara.inverse() * retPoint4d;
					vert.push_back(localPoint(0));
					vert.push_back(localPoint(1));
					vert.push_back(localPoint(2));
					intensity.push_back(reflectance);
				}
			}
		}
	}
	if (st == sensor_type::PROF_EVAL) {
		int oneLinePoint = 2000;
		int lineNumber = 1000;
		double T = 50.0;
		double tstep = T / (lineNumber);
		double tstep_ = T / (lineNumber*oneLinePoint);
		for (double t = 0;t < T;t += tstep) {
			for (double t_m = 0;t_m < tstep;t_m += tstep_) {
				//レーザーのfire direction
				Vector3d drc_zf;
				drc_zf << sin(2 * M_PI*t_m / tstep)*cos(t*M_PI / T),
					sin(2 * M_PI*t_m / tstep)*sin(M_PI*t / T),
					cos(2 * M_PI*t_m / tstep);//zf local		
				Vector3d drc_gl = campara.block(0, 0, 3, 3)*drc_zf;
				//drc_gl = drc_gl - scannerCenter;

				Vector3d retPoint;
				double reflectance;
				if ((reflectance = isearch->query_ray(scannerCenter, drc_gl, retPoint)) >= 0 && reflectance <= 1.0 && (reflectance == reflectance)) {
					Vector4d retPoint4d;
					retPoint4d << retPoint(0), retPoint(1), retPoint(2), 1;
					Vector4d localPoint = campara.inverse() * retPoint4d;
					vert.push_back(localPoint(0));
					vert.push_back(localPoint(1));
					vert.push_back(localPoint(2));
					intensity.push_back(reflectance);
				}
				else {
					vert.push_back(0);
					vert.push_back(0);
					vert.push_back(0);
					intensity.push_back(0);
				}
			}
		}

	}
}