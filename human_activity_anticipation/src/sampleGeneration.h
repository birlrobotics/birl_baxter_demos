
#include <cstdlib>
#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <dirent.h>
#include "constants.h"
#include <pcl/point_types.h>
#include "includes/point_types.h"
#include <pcl/filters/voxel_grid.h>

//#include "frame.h"

typedef pcl::PointXYZRGB PointT;

using namespace std;
using namespace pcl;

#define sqr(x) ((x)*(x))

using namespace std;

double getDistanceSqrBwPointsl(pcl::PointXYZRGB &p1, pcl::PointXYZ &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}

double getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZRGB &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}

void getSamplePoints(pcl::PointXYZ p, float size, float sr, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud ){
	int n = (int)size/sr;
	int numPoints = pow(n,3);
	int index = 0;
	//cloud.points.resize(numPoints);
	float x = p.x - size/2;
	float y = p.y - size/2;
	float z = p.z - size/2;
	cout << "head (" << p.x << ", " << p.y << "," << p.z << ")" << endl;
	cloud.width    = numPoints ;
	cloud.height   = 1;
	cloud.is_dense = false;
	cloud.points.resize (cloud.width * cloud.height);
	for(size_t i = 0; i < n; i ++){
		x += sr;
		for(size_t j = 0; j < n; j ++){
			y +=  sr;
			for(size_t k = 0; k < n; k ++){
				z += sr;
				pcl::PointXYZRGBScore np;
				np.x = x;
				np.y = y;
				np.z = z;
				np.rgb = 0;
				np.score = 0;
				cloud.points.at(index) = np;
				if(index % 10000 == 0){
			//	cout << "point " << index << " : (" << np.x << ", " << np.y << "," << np.z << ")" << endl;
				}
				index++;
			}

			z = p.z - size/2;
		}
		y = p.y - size/2;
	}
}

void addSamplePoints(pcl::PointXYZ p, float size, float sr, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud, bool top ){
	int n = (int)size/sr;
	int numPoints = pow(n,3);

	//cloud.points.resize(numPoints);
	float x = p.x - size/2;
	float y = p.y - size/2;
	float z = p.z - size/2;
	if(top) {z = p.z;}
	cout << "head (" << p.x << ", " << p.y << "," << p.z << ")" << endl;

	for(size_t i = 0; i < n; i ++){
		x += sr;
		for(size_t j = 0; j < n; j ++){
			y +=  sr;
			for(size_t k = 0; k < n; k ++){
				z += sr;
				pcl::PointXYZRGBScore np;
				np.x = x;
				np.y = y;
				np.z = z;
				np.rgb = 0;
				np.score = 0;
				cloud.points.push_back(np);

			}

			z = p.z - size/2;
			if(top) {z = p.z;}
		}
		y = p.y - size/2;
	}
	cloud.width    = cloud.points.size();
	cloud.height   = 1;
	cloud.is_dense = false;
}

 bool checkObjectProximity(Frame &f, pcl::PointXYZRGB point){
	 float minDist = 100000;
	 for(size_t i = 0; i < f.objects.size(); i ++){
		 for(size_t j = 0; j < f.objects.at(i).pcInds.size(); j++){
			 int index = f.objects.at(i).pcInds.at(j);
			 float dist = getDistanceSqrBwPoints(point, f.cloud.points.at(index));
		 	 if(dist < minDist) {minDist = dist;}
		 }
	 }
	 if (minDist < 225) {return true;}
	 return false;
 }


void getPlacingSamplePoints(Frame & f, float minD, float maxD, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud, int objId ){


	pcl::PointXYZ joint = f.skeleton.transformed_joints.at(2);
	pcl::PointCloud<pcl::PointXYZRGB> tmpCloud;
	for(size_t i = 0; i < f.cloud.points.size(); i ++){
		float dist = sqrt(getDistanceSqrBwPointsl( f.cloud.points.at(i), joint));
		if(dist > minD && dist < maxD ){
			// check if it is not too close to an object..
			if(!checkObjectProximity(f,f.cloud.points.at(i))){
				// add them to the filtered cloud
				tmpCloud.points.push_back(f.cloud.points.at(i));
			}
		}
	}
	tmpCloud.width    = tmpCloud.points.size() ;
	tmpCloud.height   = 1;
	tmpCloud.is_dense = false;

	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr ocloud (new pcl::PointCloud<pcl::PointXYZRGB> (tmpCloud));
	pcl::PointCloud<pcl::PointXYZRGB> fcloud;
	pcl::VoxelGrid<pcl::PointXYZRGB > sor;
	sor.setInputCloud (ocloud);
	sor.setLeafSize (5.0, 5.0, 5.0);
	sor.filter (fcloud);
	*/


	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr incloud (new pcl::PointCloud<pcl::PointXYZRGB> (tmpCloud));
	ne.setInputCloud (incloud);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	//pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch (10);
	ne.compute (*cloud_normals);

	for(size_t i = 0; i < tmpCloud.points.size(); i ++){
		if(cloud_normals->points.at(i).normal[2] > 0.9 || cloud_normals->points.at(i).normal[2] < -0.9){
			pcl::PointXYZRGBScore np;
			np.x = tmpCloud.points.at(i).x;
			np.y = tmpCloud.points.at(i).y;
			np.z = tmpCloud.points.at(i).z;
			np.rgb = tmpCloud.points.at(i).rgb;
			np.score = 0;
			cloud.points.push_back(np);
		}
	}
	cloud.width    = cloud.points.size() ;
	cloud.height   = 1;
	cloud.is_dense = false;
	// for all objects as the same type as objId .. add points above the object
	for(size_t i = 0; i < f.objects.size(); i ++){
		if(i == objId) {continue;}
		if(f.objects.at(i).objectType.compare(f.objects.at(objId).objectType) == 0){
			addSamplePoints(f.objects.at(i).getCentroid(), 100, 2, cloud,true);
		}
	}
	// for all containable objects add points around the object centroid
	for(size_t i = 0; i < f.objects.size(); i ++){
		if(i == objId) {continue;}
		if(f.objects.at(i).objectType.compare("microwave") == 0){
			addSamplePoints(f.objects.at(i).getCentroid(), 100, 2, cloud,false);
		}
	}


}

