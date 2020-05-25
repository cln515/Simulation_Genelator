#include "basicPly/BasicPly.h"
#include "polygonSearch.h"
#pragma once

namespace LRF {

	class LiDAR {
	public:
		int scanpersec;
		virtual void getRayDirection(double t, Matrix4d campara, std::vector<Vector3d>& laserOrigin, std::vector<Vector3d>& laserDirection) {};
	
	};

	class PanoramaSensor : public LiDAR {
	public:
		int oneLinePoint = 2862;
		int lineNumber = 1111;
		double T = 50.0;//duration for one scan
		void getRayDirection(double t, Matrix4d campara, std::vector<Vector3d>& laserOrigin, std::vector<Vector3d>& laserDirection) {
			Vector3d drc_zf, origin; origin = campara.block(0, 3, 3, 1);
			drc_zf << sin(2 * M_PI*t*lineNumber / T)*cos(M_PI*t / T), sin(2 * M_PI*t*lineNumber / T)*sin(t*M_PI / T), cos(2 * M_PI*t*lineNumber / T);//zf local		
			Vector3d drc_gl = campara.block(0, 0, 3, 3)*drc_zf;
			laserOrigin.push_back(origin);
			laserDirection.push_back(drc_gl);
		}
	};

	class MultiBeam : public LiDAR {
	public:
		
		MultiBeam() {
			scanpersec = 20000;//20000 * 16 = 320000 p/s	
			std::cout << "multi beam" << std::endl;
		}

		double T = 0.1;

		int laserCnt = 16;
		int angleMin = -15.0;
		int angleMax =  15.0;

		void getRayDirection(double t, Matrix4d campara, std::vector<Vector3d>& laserOrigin, std::vector<Vector3d>& laserDirection) {
			//レーザーのfire direction
			double horizAngle = 2 * M_PI * t / T;
			for (int i = 0; i < laserCnt; i++) {
				Vector3d drc_zf, origin; origin = campara.block(0, 3, 3, 1);
				double vertAngle = (90 - ((angleMax-angleMin) / (laserCnt-1) * i + angleMin))* M_PI / 180;
				drc_zf << sin(vertAngle)*cos(horizAngle), sin(vertAngle)*sin(horizAngle), cos(vertAngle);//zf local	)*sin(t*M_PI / T), cos(vertAngle), 1;//velo local				
				Vector3d drc_gl = campara.block(0, 0, 3, 3)*drc_zf;
				//drc_gl = drc_gl - scannerCenter;
				laserOrigin.push_back(origin);
				laserDirection.push_back(drc_gl);
				//std::cout << origin.transpose() << "," << drc_gl.transpose() << std::endl;
			}
		}
	};


	class LRF_emulator {
	public:



		struct ScanPoint {
			float x, y, z, intensity, ts;
		};

		enum sensor_type {
			ZF_IMAGER,
			HDL_64E,
			VLP_16,
			PROF_EVAL
		};

		sensor_type st;

		LiDAR* lidar;

		LRF_emulator(sensor_type st) {
			if (st == ZF_IMAGER) {
				lidar = new PanoramaSensor();
			}
			else if(st == VLP_16){
				lidar = new MultiBeam();
			}
		}

		void scanEmulation(Matrix4d campara, vector<float>& vert, vector<float>& intensity);

		//void setIsearcher(BasicPly modelData) {
		//	float* vp = modelData.getVertecesPointer();
		//	unsigned int* fp = modelData.getFaces();
		//	//zfデータの作成
		//	isearch.buildTree(vp, fp, modelData.getVertexNumber(), modelData.getFaceNumber());//AABBtreeの作成

		//}
		//void setReflectance(BasicPly modelData) {
		//	float* rf = modelData.getReflectancePointer();
		//	unsigned int* fp = modelData.getFaces();
		//	isearch.setReflectance(rf, modelData.getVertexNumber());
		//}

		void setIntersectionSearcher(IntersectionSearcher* isearch_) {
			isearch = isearch_;
		};

		std::vector<ScanPoint> scan(double t, Matrix4d campara);
		std::string fileBase;

	private:
		IntersectionSearcher* isearch;;

	};
};