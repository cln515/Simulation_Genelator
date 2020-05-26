#include <FusionPlatform.h>


void FusionPlatform::scan(double ts, double te) {

	for (int i = 0; i < cameras.size(); i++) {
		double exposure = (1.0 / cameras.at(i)->fps);
		int ts_cnt = ceil(ts / exposure);
		cameras.at(i)->createContext();
		Matrix4d extParam = ext_camera.at(i);
		std::vector<std::string> fileNameList;
		std::vector<_6dof> motion;
		std::vector<double> timestamp;
		std::cout << ts << "," << te << "," << ts_cnt << "," << exposure << std::endl;
		for (double t = exposure * ts_cnt; t <= te; t += exposure) {
			Vector4d q;
			q(0) = sqx(t);
			q(1) = sqy(t);
			q(2) = sqz(t);
			q(3) = sqw(t);
			q.normalize();
			std::cout << q.transpose() << std::endl;

			Vector3d pos;
			pos(0) = sx(t);
			pos(1) = sy(t);
			pos(2) = sz(t);
			Matrix4d campara, cinv,cpara;
			campara.block(0, 0, 3, 3) = q2dcm(q);
			campara.block(0, 3, 3, 1) = pos;
			campara.block(3, 0, 1, 4) << 0, 0, 0, 1;
			cinv =(campara* extParam).inverse();
			std::cout << campara << std::endl << std::endl;
			cameras.at(i)->renderColor(cinv);//multiply campara
			std::string filepath = cameras.at(i)->imwrite(),fname;
			fname = filepath.substr(filepath.find_last_of("\\") + 1);
			fileNameList.push_back(fname);
			cameras.at(i)->clearImage();

			cpara = cinv.inverse();
			_6dof mot_dof = m2_6dof(cpara);
			motion.push_back(mot_dof);
			timestamp.push_back(t);
		}
		cameras.at(i)->discardContext();
		//output filename list, motion, timestamp
		ofstream ofslist(cameras.at(i)->fileBase + ".lst");
		ofstream ofsmotion(cameras.at(i)->fileBase + "_l.dat",std::ios::binary);
		ofstream ofsts(cameras.at(i)->fileBase + "_ts.dat", std::ios::binary);
		int frameNum = fileNameList.size();
		ofslist << frameNum << std::endl;
		for (int j = 0; j < fileNameList.size(); j++) {
			ofslist << fileNameList.at(j)<<std::endl;
		}

		ofsmotion.write((char*)&frameNum,sizeof(int));
		ofsmotion.write((char*)motion.data(), sizeof(_6dof)*motion.size());
		ofsts.write((char*)&frameNum, sizeof(int));
		ofsts.write((char*)timestamp.data(), sizeof(double)*timestamp.size());
	}

	for (int i = 0; i < lidars.size(); i++) {
		__int64 t_start = ceil( ts / (1.0 / lidars.at(i)->lidar->scanpersec)),
			t_end =floor( te / (1.0 / lidars.at(i)->lidar->scanpersec));
		std::cout << ts << "," << te << "," << lidars.at(i)->lidar->scanpersec << std::endl;
		std::vector<LRF::LRF_emulator::ScanPoint> scans_;
		Matrix4d extParam = ext_Lidar.at(i);
		std::cout << t_start << "," << t_end << std::endl;
		std::vector<float> world_vert;
		std::vector<float> world_intensity;
		for (int j = t_start; j < t_end; j++) {
			double t = 1.0 * j / lidars.at(i)->lidar->scanpersec;
			Vector4d q;
			q(0) = sqx(t);
			q(1) = sqy(t);
			q(2) = sqz(t);
			q(3) = sqw(t);
			q.normalize();
			Vector3d pos;
			pos(0) = sx(t);
			pos(1) = sy(t);
			pos(2) = sz(t);
			Matrix4d campara, cinv,cpara;
			campara.block(0, 0, 3, 3) = q2dcm(q);
			campara.block(0, 3, 3, 1) = pos;
			campara.block(3, 0, 1, 4) << 0, 0, 0, 1;
			cpara = campara * extParam;

			std::vector<LRF::LRF_emulator::ScanPoint> scans = lidars.at(i)->scan(t, cpara);
			scans_.insert(scans_.end(), scans.begin(), scans.end());
			for (int k = 0; k < scans.size(); k++) {
				Vector4d pt;
				pt<<scans.at(k).x, scans.at(k).y , scans.at(k).z,1;
				if (pt.norm() <= 1)continue;
				pt = cpara * pt;
				world_vert.push_back(pt(0));
				world_vert.push_back(pt(1));
				world_vert.push_back(pt(2));
				world_intensity.push_back(scans.at(k).intensity);
			}
		}
		ofstream ofs(lidars.at(i)->fileBase + ".dat", std::ios::binary);
		__int64 vnum = scans_.size();
		ofs.write((char*)&vnum,sizeof(__int64));
		ofs.write((char*)scans_.data(), sizeof(LRF::LRF_emulator::ScanPoint)*scans_.size());
		;
		BasicPly bp;
		bp.setVertecesPointer(world_vert.data(), world_vert.size() / 3);
		bp.setReflectancePointer(world_intensity.data(), world_vert.size() / 3);
		bp.writePlyFileAuto(lidars.at(i)->fileBase + "_world.ply");
	}




}

void FusionPlatform::setTime(std::vector<double> time, std::vector<Matrix4d> motion) {
	std::vector<double> px, py, pz, pqx, pqy, pqz, pqw;
	Vector4d qprev;
	for (int i = 0; i < motion.size(); i++) {
		Matrix3d r = motion.at(i).block(0,0,3,3);
		Vector4d q = dcm2q(r);
		if (i!=0 && q.dot(qprev) < 0)q = qprev;
		Vector3d t = motion.at(i).block(0, 3, 3, 1);
		px.push_back(t(0));
		py.push_back(t(1));
		pz.push_back(t(2));
		pqx.push_back(q(0));
		pqy.push_back(q(1));
		pqz.push_back(q(2));
		pqw.push_back(q(3));
		std::cout << q.transpose() << std::endl;
		qprev = q;
	}
	sx.set_points(time, px);
	sy.set_points(time, py);
	sz.set_points(time, pz);
	sqx.set_points(time, pqx);
	sqy.set_points(time, pqy);
	sqz.set_points(time, pqz);
	sqw.set_points(time, pqw);
}