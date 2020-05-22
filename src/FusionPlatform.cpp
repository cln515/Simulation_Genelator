#include <FusionPlatform.h>


void FusionPlatform::scan(double ts, double te) {

	for (int i = 0; i < cameras.size(); i++) {
		double exposure = (1.0 / cameras.at(i)->fps);
		int ts_cnt = ts / exposure;
		for (double t = exposure * ts_cnt + 1; t < te; t += exposure) {
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
			Matrix4d campara, cinv;
			campara.block(0, 0, 3, 3) = q2dcm(q);
			campara.block(0, 3, 3, 1) = pos;
			campara.block(3, 0, 1, 4) << 0, 0, 0, 1;
			cinv = campara.inverse();
			std::cout << campara << std::endl << std::endl;
			cameras.at(i)->renderColor(cinv);//multiply campara
			cameras.at(i)->imwrite();
			cameras.at(i)->clearImage();
		}
	}

	for (int i = 0; i < lidars.size(); i++) {
		__int64 t_start = ts / (1.0 / lidars.at(i)->lidar->scanpersec) + 1,
			t_end = te / (1.0 / lidars.at(i)->lidar->scanpersec) - 1;
		std::cout << ts << "," << te << "," << lidars.at(i)->lidar->scanpersec << std::endl;
		std::vector<LRF::LRF_emulator::ScanPoint> scans_;
		std::cout << t_start << "," << t_end << std::endl;
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
			Matrix4d campara, cinv;
			campara.block(0, 0, 3, 3) = q2dcm(q);
			campara.block(0, 3, 3, 1) = pos;
			campara.block(3, 0, 1, 4) << 0, 0, 0, 1;
			cinv = campara.inverse();

			std::vector<LRF::LRF_emulator::ScanPoint> scans = lidars.at(i)->scan(t, campara);
			scans_.insert(scans_.end(), scans.begin(), scans.end());
		}
		ofstream ofs("pc.dat", std::ios::binary);
		ofs.write((char*)scans_.data(), sizeof(LRF::LRF_emulator::ScanPoint)*scans_.size());
		;
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