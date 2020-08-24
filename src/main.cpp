

#include "basicPly/BasicPly.h"
#include "image_utility/image_utility.h"
#include "nlohmann/json.hpp"
#include <iostream>
#include "FusionPlatform.h"
#include "LBEmulator.h"
#include "LRF_emu.h"
#include "PanoramaRenderer/PanoramaRenderer.h"



int main(int argc, char *argv[])
{

	std::ifstream i(argv[1]);
	nlohmann::json config;
	i >> config;


	string input=config["input"].get<std::string>();
	string motf = config["motion"].get<std::string>();
	std::list<nlohmann::json> cameras = config["Camera"].get<std::list<nlohmann::json>>();
	std::list<nlohmann::json> lidars = config["LiDAR"].get<std::list<nlohmann::json>>();
	double scan_start = config["scan_start"].get<double>();
	double scan_end = config["scan_end"].get<double>();
	string outputFolder = config["output"].get<std::string>();
	makeFolder(outputFolder + "/");
	ifstream ifsmot(motf);
	string str;
	vector<Matrix4d> motiondatas;
	vector<double> motiontimes;



//ply file loading
	BasicPly modelData;
	modelData.readPlyFile_(input);


	//set sensors
	FusionPlatform fp;
	//set camera
	std::list<nlohmann::json>::iterator cameraconf;	 
	for (cameraconf = cameras.begin(); cameraconf != cameras.end(); cameraconf++) {
		camera* pr = new camera();
		pr->setDataRGB(modelData.getVertecesPointer(), modelData.getFaces(), modelData.getRgbaPointer(), modelData.getVertexNumber(), modelData.getFaceNumber());
		std::string camType = (*cameraconf)["type"].get<std::string>();
		if (camType.compare("Fisheye") == 0) {
			pr->setFisheye();
			pr->setUniqueViewSize((*cameraconf)["width"].get<int>(), (*cameraconf)["height"].get<int>());
			int viewWidth_, viewHeight_;
			pr->getUniqueViewSize(viewWidth_, viewHeight_);
			cv::Mat mask = cv::Mat::zeros(cv::Size(viewWidth_, viewHeight_), CV_8UC1);
			cv::ellipse(mask, cv::RotatedRect(cv::Point2f(viewWidth_ / 2, viewHeight_ / 2), cv::Size2f(viewWidth_, viewHeight_), 0), cv::Scalar(255.0), -1);
			cv::imwrite(outputFolder + "/mask.png",mask);
		}
		else if (camType.compare("Panorama") == 0) {
			pr->setUniqueViewSize((*cameraconf)["width"].get<int>(), (*cameraconf)["height"].get<int>());
		}
		else {
			pr->setPersRender((*cameraconf)["width"].get<int>() / 2, (*cameraconf)["height"].get<int>() / 2, (*cameraconf)["focal"].get<int>(), (*cameraconf)["focal"].get<int>());//Todo set from input file
			pr->setUniqueViewSize((*cameraconf)["width"].get<int>(), (*cameraconf)["height"].get<int>());
		}
		pr->fileBase = outputFolder +"/"+ (*cameraconf)["file"].get < std::string >();
		makeFolder(pr->fileBase);

		_6dof _ext = { (*cameraconf)["rx"].get<double>() / 180.0*M_PI
		, (*cameraconf)["ry"].get<double>() / 180.0*M_PI
		, (*cameraconf)["rz"].get<double>() / 180.0*M_PI
		, (*cameraconf)["x"].get<double>()
		, (*cameraconf)["y"].get<double>()
		, (*cameraconf)["z"].get<double>() };
		Matrix4d extParam = _6dof2m(_ext);

		fp.cameras.push_back(pr);
		fp.ext_camera.push_back(extParam);

		(*cameraconf)["cx"] = (*cameraconf)["width"].get<int>() / 2;
		(*cameraconf)["cy"] = (*cameraconf)["height"].get<int>() / 2;
		(*cameraconf)["f"] = (*cameraconf)["width"].get<int>() /M_PI;

	
	}
	//set lidar
	IntersectionSearcher* isearch = new IntersectionSearcher();
	{
		float* vp = modelData.getVertecesPointer();
		unsigned int* fp = modelData.getFaces();
		isearch->buildTree(vp, fp, modelData.getVertexNumber(), modelData.getFaceNumber());//AABBtreeの作成
	}
	
	std::list<nlohmann::json>::iterator lidarconf;
	for (lidarconf = lidars.begin(); lidarconf != lidars.end(); lidarconf++) {
		LRF::LRF_emulator::sensor_type sensortype;
		std::string lidType =(*lidarconf)["type"].get<std::string>();
		if (lidType.compare("VLP-16") == 0) {
			sensortype = LRF::LRF_emulator::sensor_type::VLP_16;
		}
		else if (lidType.compare("ZF-IMAGER") == 0) {
			sensortype = LRF::LRF_emulator::sensor_type::ZF_IMAGER;
		}//custom lidar
		std::cout << "sensor type:" << sensortype << "," << lidType << std::endl;
		LRF::LRF_emulator* lidartest = new LRF::LRF_emulator(sensortype);
		lidartest->setIntersectionSearcher(isearch);
		_6dof _ext = { (*lidarconf)["rx"].get<double>()/180.0*M_PI
			, (*lidarconf)["ry"].get<double>() / 180.0*M_PI
			, (*lidarconf)["rz"].get<double>() / 180.0*M_PI
			, (*lidarconf)["x"].get<double>()
			, (*lidarconf)["y"].get<double>()
			, (*lidarconf)["z"].get<double>() };
		Matrix4d extParam = _6dof2m(_ext);

		lidartest->fileBase = outputFolder + "/" + (*lidarconf)["file"].get < std::string >();
		makeFolder(lidartest->fileBase);

		fp.lidars.push_back(lidartest);
		fp.ext_Lidar.push_back(extParam);
	}

	while (getline(ifsmot, str)) {
		double motdata[9];
		double time;
		for (int i = 0; i < 10; i++) {
			if (i == 0) {
				time = stod(str.substr(0, str.find_first_of(" ")));
			}else if (i != 9) {
				motdata[i-1] = stod(str.substr(0, str.find_first_of(" ")));
			}
			else{
				motdata[i - 1] = stod(str);
			}
			str.erase(0, str.find_first_of(" ") + 1);
			
		}
		Matrix4d m1 = lookat2matrix(motdata);
		motiondatas.push_back(m1);
		cout << m1 << endl;
		motiontimes.push_back(time);
	}
	ifsmot.close();

	fp.setTime(motiontimes, motiondatas);
	fp.scan(scan_start, scan_end);
	//ext parameter output
	for (int i = 0; i < fp.ext_camera.size(); i ++ ) {
		for (int j = 0; j < fp.ext_Lidar.size(); j++) {
			Matrix4d extCpara = fp.ext_Lidar.at(j).inverse() * fp.ext_camera.at(i);
			std::string fileName = outputFolder + "/" + "lid"+std::to_string(j)+"-cam" + std::to_string(i) +".cpara";
			writeCPara(fileName, extCpara);
		}
	}

	//json output
	nlohmann::json outj;
	outj["work_folder"] = outputFolder;
	outj["depth_folder"] = outputFolder + "/depth";
	outj["cpara"] = outputFolder + "/lid0-cam0.cpara";
	outj["scan"] = outputFolder + "/point/binary";
	outj["image"] = outputFolder + "/image";
	outj["image_prefix"] = "image";
	outj["sfm_motion"] = outputFolder + "/camera.xml";
	outj["sfm_model"] = outputFolder + "/model.ply";
	outj["camera_param"] = cameras;
	outj["scan_pointcloud"] = outputFolder + "/comp_scan3.ply";
	outj["motion"] = outputFolder + "/motion.dat";
	outj["first_frame"] = 0;
	outj["last_frame"] = fp.frameCnt - 1 ;
	outj["mode"] = 2;

	outj["output"] = outputFolder + "/output";
	outj["time_threshold"] = 10.0;
	outj["range"] = "0.5-8.0";
	outj["skip"] = 1;
	outj["image_downsample"] = 5;
//	outj["spec"] = true;
	outj["mask"] = outputFolder + "/mask.png";




	ofstream ofsjson(outputFolder+"/process.json");
	ofsjson << std::setw(4) << outj << std::endl;
	ofsjson.close();

	ofsjson.close();



    std::cout << "Hello World!\n"; 
}

