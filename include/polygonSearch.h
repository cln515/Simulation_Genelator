#include <iostream>
#include <list>
#include <algorithm>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <ann/ANN.h>
#include "Eigen/Eigen"
#include "Eigen/Core"
typedef CGAL::Simple_cartesian<double> K;
typedef K::FT FT;
typedef K::Ray_3 Ray;
typedef K::Line_3 Line;
typedef K::Segment_3 seg;
typedef K::Point_3 Point;
typedef K::Triangle_3 Triangle;
typedef std::list<Triangle>::iterator Iterator;
typedef CGAL::AABB_triangle_primitive<K, Iterator> Primitive;
typedef CGAL::AABB_traits<K, Primitive> AABB_triangle_traits;
typedef CGAL::AABB_tree<AABB_triangle_traits> Tree;
typedef Tree::Primitive_id Primitive_id;

using namespace Eigen;



class IntersectionSearcher{
public:
	void buildTree(float* vertex,unsigned int* mesh,int vertNum,int meshNum);
	float query_ray(Vector3d origin,Vector3d direction,Vector3d& out);
	void setReflectance(float* ref, unsigned int vertNum);
private:
	Tree* tree;
	std::list<Triangle> triangles;
	std::vector<Point> points;
	std::vector<float> reflectance;
	bool reflectanceSet=false;

	ANNpointArray dataPts = NULL;
	ANNkd_tree* kdtree = NULL;
};