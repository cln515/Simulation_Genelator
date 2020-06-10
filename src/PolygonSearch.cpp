#include "polygonSearch.h"


void IntersectionSearcher::buildTree(float* vertex,unsigned int* mesh,int vertNum,int meshNum){
	
	for(int i=0;i<vertNum;i++){
		points.push_back(Point(vertex[i*3],vertex[i*3+1],vertex[i*3+2]));	
	}


	
	triangles.clear();

	for(int i=0;i<meshNum;i++){
		triangles.push_back(Triangle(points.at(mesh[i*3]),
			points.at(mesh[i*3+1]),
			points.at(mesh[i*3+2])));	
	}
	tree=new Tree(triangles.begin(),triangles.end());
	//tree->build();
	
	Point o(0,0,0);
	K::Direction_3 v(0,0,1);
    Ray ray_query(o,v);

	std::list<boost::optional< Tree::Intersection_and_primitive_id<seg>::Type>> intersections;
	tree->all_intersections(ray_query,std::back_inserter(intersections));
}

float IntersectionSearcher::query_ray(Vector3d origin,Vector3d direction,Vector3d& out){
	
	Point o(origin(0),origin(1),origin(2));
	K::Direction_3 v(direction(0),direction(1),direction(2));
    Ray ray_query(o,v);

	std::list<boost::optional< Tree::Intersection_and_primitive_id<seg>::Type>> intersections;
	tree->all_intersections(ray_query,std::back_inserter(intersections));
	//auto fip = tree->first_intersection(query_ray);
//	boost::optional< Tree::Intersection_and_primitive_id<Triangle>::Type> n=tree->first_intersected_primitive(query_ray);
	
	if(intersections.size()==0)return -1;
	std::list<boost::optional< Tree::Intersection_and_primitive_id<seg>::Type>>::iterator it=intersections.begin();

	double min=40000;
	Point vtx1, vtx2, vtx3;
	while(it != intersections.end()){
		Point* p=boost::get<Point>(&(*it)->first);
		if (p != NULL) {
			double d = CGAL::squared_distance(o, (*p));
			if (d < min) {
				min = d;
				out(0) = (*p).x();
				out(1) = (*p).y();
				out(2) = (*p).z();
				vtx1 = (*it)->second->vertex(0);
				vtx2 = (*it)->second->vertex(1);
				vtx3 = (*it)->second->vertex(2);
			}
		}
		++it;
	}
	if (reflectanceSet) {
		ANNpoint queryPt = annAllocPt(3);
		ANNidxArray nnidx = new ANNidx[1];
		ANNdistArray dists = new ANNdist[1];
		queryPt[0] = vtx1.x();
		queryPt[1] = vtx1.y();
		queryPt[2] = vtx1.z();
		kdtree->annkSearch(queryPt, 1, nnidx, dists);
		float r1 = reflectance.at(nnidx[0]);
		//std::vector<Point>::iterator vItr1 = std::find(points.begin(), points.end(), vtx1);
		//size_t idx1 = std::distance(points.begin(), vItr1);
		//std::vector<Point>::iterator vItr2 = std::find(points.begin(), points.end(), vtx2);
		//size_t idx2 = std::distance(points.begin(), vItr2);
		//std::vector<Point>::iterator vItr3 = std::find(points.begin(), points.end(), vtx3);
		//size_t idx3 = std::distance(points.begin(), vItr3);
		queryPt[0] = vtx2.x();
		queryPt[1] = vtx2.y();
		queryPt[2] = vtx2.z();
		kdtree->annkSearch(queryPt, 1, nnidx, dists);

		float r2 = reflectance.at(nnidx[0]);
		queryPt[0] = vtx3.x();
		queryPt[1] = vtx3.y();
		queryPt[2] = vtx3.z();
		kdtree->annkSearch(queryPt, 1, nnidx, dists);
		float r3 = reflectance.at(nnidx[0]);

		double areas[3];
		Vector3d vtx1_, vtx2_, vtx3_;
		vtx1_ << vtx1.x(), vtx1.y(), vtx1.z();
		vtx2_ << vtx2.x(), vtx2.y(), vtx2.z();
		vtx3_ << vtx3.x(), vtx3.y(), vtx3.z();
		areas[0] = ((vtx2_-out).cross(vtx3_ - out)).norm();
		areas[1] = ((vtx3_ - out).cross(vtx1_ - out)).norm();
		areas[2] = ((vtx1_ - out).cross(vtx2_ - out)).norm();

		double sum = areas[0] + areas[1] + areas[2];

		delete queryPt;
		delete nnidx;
		delete dists;
		double refl = (r1*areas[0] + r2 * areas[1] + r3 * areas[2]) / sum;

		if (refl < 0 || refl> 1.0 || refl!=refl) {
			std::cout << areas[0] << "," << areas[1] << "," << areas[2] << std::endl;
			std::cout << refl << std::endl;
			std::cout <<r1 <<","<<r2<<","<<r3 << std::endl;
		}

		return refl;

	}
	else {
		return 0.8;
	}

	//std::list<Primitive_id> primitives;
	//tree->all_intersected_primitives(ray_query, std::back_inserter(primitives));
	//std::list<Primitive_id>::iterator it2 = primitives.begin();

	//double min = 40000;
	//while (it2 != primitives.end()) {
	//	&(*it2)->vertex;
	//	Point* p = boost::get<Point>(&(*it)->first);
	//	double d = CGAL::squared_distance(o, (*p));
	//	if (d<min) {
	//		min = d;
	//		out(0) = (*p).x();
	//		out(1) = (*p).y();
	//		out(2) = (*p).z();
	//	}
	//	++it2;
	//}



}

void IntersectionSearcher::setReflectance(float* refp, unsigned int vertNum) {
	for (int i = 0;i<vertNum;i++) {
		reflectance.push_back(refp[i]);
	}






	dataPts = annAllocPts(points.size(), 3);
	for (int i_ = 0;i_<points.size();i_++) {
		dataPts[i_][0] = points.at(i_).x();
		dataPts[i_][1] = points.at(i_).y();
		dataPts[i_][2] = points.at(i_).z();
	}
	kdtree = new ANNkd_tree(dataPts, points.size(), 3);
	

	reflectanceSet = true;
	return;
}