
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
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <pcl/point_types.h>
#include "includes/point_types.h"
#include "includes/CombineUtils.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/discrete_distribution.hpp>
#include "includes/bezier.h";
//#include "frame.h"

typedef pcl::PointXYZRGB PointT;

using namespace std;
using namespace pcl;

#define sqr(x) ((x)*(x))

using namespace std;


vector<PointXYZ> getSLTrajPoints(pcl::PointXYZ startPoint,pcl::PointXYZRGBScore endPoint, int length){
	vector<PointXYZ> trajPoints;
	for(int i = 0; i < length  ; i ++){
		PointXYZ p;
		p.x = startPoint.x+ i*(endPoint.x-startPoint.x)/length;
		p.y = startPoint.y+ i*(endPoint.y-startPoint.y)/length;
		p.z = startPoint.z+ i*(endPoint.z-startPoint.z)/length;
		trajPoints.push_back(p);
	}
	return trajPoints;
		    	
}

vector<PointXYZ> getCleaningTrajPoints(pcl::PointXYZ startPoint,pcl::PointXYZRGBScore endPoint, int length, Frame f, int objid ){
	vector<PointXYZ> trajPoints;
	trajPoints.push_back(startPoint);
	for(int i = 1; i < length -1  ; i ++){
		PointXYZ p;
		int index = rand()% f.objects.at(objid).pcInds.size();

		p.x = f.cloud.at(index).x;
		p.y = f.cloud.at(index).y;
		p.z = f.cloud.at(index).z;
		trajPoints.push_back(p);
	}
	PointXYZ p(endPoint.x, endPoint.y, endPoint.z);
	trajPoints.push_back(p);
	return trajPoints;

}

vector<PointXYZ> getStationaryTrajPoints(pcl::PointXYZ startPoint, int length){
	vector<PointXYZ> trajPoints;
	for(int i = 0; i < length  ; i ++){

		trajPoints.push_back(startPoint);
	}
	return trajPoints;

}

	TransformG createZTransform(float dx, float dy, float dz, float a){
		vector<double> m ;
		m.push_back(cos(a));
		m.push_back(-sin(a));
		m.push_back(0);
		m.push_back(0);

		m.push_back(sin(a));
		m.push_back(cos(a));
		m.push_back(0);
		m.push_back(0);

		m.push_back(0);
		m.push_back(0);
		m.push_back(1);
		m.push_back(0);

		m.push_back(0);
		m.push_back(0);
		m.push_back(0);
		m.push_back(1);
		TransformG t(m);
		return t;
	}

	TransformG createYTransform(float dx, float dy, float dz, float a){
		vector<double> m ;
		m.push_back(cos(a));
		m.push_back(0);
		m.push_back(sin(a));
		m.push_back(0);

		m.push_back(0);
		m.push_back(1);
		m.push_back(0);
		m.push_back(0);

		m.push_back(-sin(a));
		m.push_back(0);
		m.push_back(cos(a));
		m.push_back(0);

		m.push_back(0);
		m.push_back(0);
		m.push_back(0);
		m.push_back(1);

		TransformG t(m);
		return t;
	}

	TransformG createScaleTransform(float s){
		vector<double> m ;
		m.push_back(s);
		m.push_back(0);
		m.push_back(0);
		m.push_back(0);

		m.push_back(0);
		m.push_back(s);
		m.push_back(0);
		m.push_back(0);

		m.push_back(0);
		m.push_back(0);
		m.push_back(s);
		m.push_back(0);

		m.push_back(0);
		m.push_back(0);
		m.push_back(0);
		m.push_back(1);

		TransformG t(m);
		return t;
	}

vector<PointXYZ> getTrajPoints(pcl::PointXYZ startPoint,pcl::PointXYZRGBScore endPoint, int length){
		float dx = endPoint.x - startPoint.x;
		float dy = endPoint.y - startPoint.y;
		float dz = endPoint.z - startPoint.z;
		float angle = atan(dy/dx);
		if(dx<0 && dy>0 || dx<0 && dy<0 )
			angle = PI-angle;
		else
			angle = -angle;
		float r = sqrt(dx*dx+dy*dy+dz*dz);
		float angle2 = PI/2 - acos(dz/r);


		TransformG t = createZTransform(dx,dy,dz,-angle);
		TransformG t2 = createYTransform(dx,dy,dz,-angle2);
		TransformG t3 = createScaleTransform(r);
		vector <pcl::PointXYZ> cp;
		cp.push_back(startPoint);
		pcl::PointXYZ cp1(0.3,0,0.4);
		cp.push_back(cp1);

		pcl::PointXYZ cp2(0.6,0,0.4);
		cp.push_back(cp2);
		pcl::PointXYZ ep(endPoint.x,endPoint.y,endPoint.z);
		cp.push_back(ep);

		for(size_t i =1; i <= 2; i ++){
			t3.transformPointInPlace(cp.at(i));
			t2.transformPointInPlace(cp.at(i));
			t.transformPointInPlace(cp.at(i));
			cp.at(i).x += startPoint.x;
			cp.at(i).y += startPoint.y;
			cp.at(i).z += startPoint.z;
		}

		// code to generate the points on the curve
		return Bezier(length,cp);
	}

