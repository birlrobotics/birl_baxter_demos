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
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include "includes/point_types.h"
#include "includes/CombineUtils.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/discrete_distribution.hpp>
//#include "includes/bezier.h";
#include "trajectory_shapes.h"
#include "sampleGeneration.h"
//#include "frame.h"

typedef pcl::PointXYZRGB PointT;

using namespace std;
using namespace pcl;

#define sqr(x) ((x)*(x))

using namespace std;

class hallucination {
private:

	map <string, string> actaffMap;
	map <string, string> interactaffMap;
	int sf,ef;


	void initializeMaps(){
		interactaffMap["reaching"] = "reachable";
		interactaffMap["pouring"] = "pourto";
		interactaffMap["cleaning"] = "cleanable";
		actaffMap["moving"] = "movable";
		actaffMap["pouring"] = "pourable";
		actaffMap["eating"] = "eatable";
		actaffMap["drinking"] = "drinkable";
		actaffMap["opening"] = "openable";
		actaffMap["placing"] = "placeable";
		actaffMap["closing"] = "closeable";
		actaffMap["null"] = "stationary";
		actaffMap["cleaning"] = "cleaner";
	}
	void saveFloatImage ( const char* filename, const IplImage * image )
	{
	  IplImage * saveImage = cvCreateImage ( cvGetSize ( image ),
	                                             IPL_DEPTH_32F, 3 );
	  cvConvertScale ( image, saveImage, 255, 0 );
	  cvSaveImage( filename, saveImage);
	  cvReleaseImage ( &saveImage );
	}

	void saveImage(pcl::PointCloud<PointT> & cloud,vector<pcl::PointXYZ> &Points, string imagename) {
	        CvSize size;
	        int thickness = 8;
	        int lineType = 8;
	        size.height = 480;
	        size.width = 640;
	        IplImage * image = cvCreateImage(size, IPL_DEPTH_32F, 3);
	        //assert(cloud.size() == size.width * size.height);
	        PointT tmp;
	        for (int x = 0; x < size.width; x++)
	            for (int y = 0; y < size.height; y++) {
	                int index = ( x) + ( y) * 640;
	                tmp = cloud.points[index];
	                ColorRGB tmpColor(tmp.rgb);
	                CV_IMAGE_ELEM(image, float, y, 3 * x) = tmpColor.b; //float(IMAGE[obj.minX + x][obj.minY + y][0]) /255 ;// tmpColor.b;
	                CV_IMAGE_ELEM(image, float, y, 3 * x + 1) = tmpColor.g; //float(IMAGE[obj.minX + x][obj.minY + y][1])/255 ; //tmpColor.g;
	                CV_IMAGE_ELEM(image, float, y, 3 * x + 2) = tmpColor.r; //float(IMAGE[obj.minX + x][obj.minY + y][2])/255 ;//tmpColor.r;
	            }

	        string filename = "image_traj_"+imagename+".png";


	        for (int i =0; i < Points.size(); i++){
	        	pcl::PointXYZ p1 = Points.at(i);

	        	//std::cout << "point: (" << p1.x << "," << p1.y << ")"  << endl;;
	        	cvCircle(image, cvPoint(int(p1.x),int(p1.y)),4, cv::Scalar( 1,0,0 ),-8);
	        }

	        //cvLine( image, cvPoint(10, 40), cvPoint(100,200 ),cv::Scalar( 1,1,1 ),thickness,lineType );
	        saveFloatImage(filename.c_str(), image);

	        cvReleaseImage(&image);


	    }

	pcl::PointXYZ getImagePoint(pcl::PointXYZ  p){
	    	PointXYZ p1;
	    	p1.x = p.x ; p1.y = p.y; p1.z = p.z;
	        globalTransform.transformPointInPlace(p1);
	        pcl::PointXYZ rp;
	        if(p1.y != 0.0){
	            rp.x = ((p1.x * (640.0/1.1147))/p1.y) + 320;
	            rp.y = 240 - ((p1.z * (480.0/0.8336))/p1.y);
	        }
	        return rp;
	    }


	   PointXYZ getImageShift(PointXYZ p1, PointXYZ p2){
	    	PointXYZ imp1 = getImagePoint(p1);
	    	PointXYZ imp2 = getImagePoint(p2);
	    	PointXYZ delta;
	    	delta.x = - imp1.x + imp2.x;
	    	delta.y = - imp1.y + imp2.y;
	    	return delta;
	    }
/*	void getSLTrajPoints(){

		    	for(int i = 0; i < length  ; i ++){
		    		PointXYZ p;
		    		p.x = startPoint.x+ i*(endPoint.x-startPoint.x)/length;
		    		p.y = startPoint.y+ i*(endPoint.y-startPoint.y)/length;
		    		p.z = startPoint.z+ i*(endPoint.z-startPoint.z)/length;
		    		trajPoints.push_back(p);
		    	}
		    	vector<pcl::PointXYZ> imgTrajPoints;
		    	for(int i = 0; i < trajPoints.size(); i ++){
		    		imgTrajPoints.push_back(getImagePoint(trajPoints.at(i)));
		    	}

		    	saveImage(startFrame.cloud, imgTrajPoints, aid+"_"+halId);
		    	//cout << "size=" << trajPoints.size() << endl;
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

	void getTrajPoints(){
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
		trajPoints = Bezier(length,cp);


}
*/

	void saveTrajectory(){
		vector<pcl::PointXYZ> imgTrajPoints;
		for(int i = 0; i < trajPoints.size(); i ++){
			imgTrajPoints.push_back(getImagePoint(trajPoints.at(i)));
		}

		saveImage(startFrame.cloud, imgTrajPoints, aid+"_"+halId);
					    	//cout << "size=" << trajPoints.size() << endl;
	}

	void createSegment(){
		cout << "creating segment" << endl;

		for (size_t i = 0; i < trajPoints.size(); i ++){
			cout << "frame " << i << endl;
			segmentHal.push_back(startFrame);
			segmentHal.at(i).frameNum = startFrame.frameNum+i+1;
		}
		cout << "size of segment: "<< segmentHal.size() << endl;
		if(objectId>=0){
			for (size_t i = 0; i < trajPoints.size(); i ++){
				vector<double> features = segmentHal.at(i).objects.at(objectId).features;
				PointXYZ shift = getImageShift(segmentHal.at(i).objects.at(objectId).getCentroid(), trajPoints.at(i));
				segmentHal.at(i).objects.at(objectId).setCentroid(trajPoints.at(i));
				// change features vector too
				features.at(2) += int(shift.x);
				features.at(4) += int(shift.x);
				features.at(3) += int(shift.y);
				features.at(5) += int(shift.y);
				//features.at(6) = 0;
				//features.at(7) = 0;
				//features.at(8) = 0;
				//features.at(9) = 0;
				features.at(10) = shift.x;
				features.at(11) = shift.y;
				segmentHal.at(i).objects.at(objectId).setFeatures(features);
			}
			if(activeHands.size()>0){ // if there are active hands.. then make the follow the relative trajectory
				for(size_t a = 0; a < activeHands.size(); a++){
					int joint = activeHands.at(a);
					float dx = segmentHal.at(0).skeleton.transformed_joints.at(joint).x - startPoint.x;
					float dy = segmentHal.at(0).skeleton.transformed_joints.at(joint).y - startPoint.y;
					float dz = segmentHal.at(0).skeleton.transformed_joints.at(joint).z - startPoint.z;
					for (size_t i = 0; i < trajPoints.size(); i ++){
						segmentHal.at(i).skeleton.transformed_joints.at(joint).x = dx + trajPoints.at(i).x;
						segmentHal.at(i).skeleton.transformed_joints.at(joint).y = dy + trajPoints.at(i).y;
						segmentHal.at(i).skeleton.transformed_joints.at(joint).z = dz + trajPoints.at(i).z;
					}
				}
			}
		}else{// hand motion only.. change the corresponding hand location
			int numJoints = segmentHal.at(0).skeleton.transformed_joints.size();
			int joint = numJoints + objectId;
			cout << "hallucinating skel joint:" << joint << endl;
			for (size_t i = 0; i < trajPoints.size(); i ++){
				segmentHal.at(i).skeleton.transformed_joints.at(joint).x = trajPoints.at(i).x;
				segmentHal.at(i).skeleton.transformed_joints.at(joint).y = trajPoints.at(i).y;
				segmentHal.at(i).skeleton.transformed_joints.at(joint).z = trajPoints.at(i).z;
			}
		}
		cout << "size of segment: "<< segmentHal.size() << endl;
		//sf = segmentHal.at(0).frameNum;
		ef = segmentHal.at(segmentHal.size()-1).frameNum;
		sf = ef - length +1 ;
	}
	void computeFeatures(){
		FrameFeatures ff(true,"hallucination_"+aid+"_"+halId);
		ff.setCurrentFrame(startFrame);
		for(size_t i = 0; i < segmentHal.size(); i ++ ){
			ff.setCurrentFrame(segmentHal.at(i));
			ff.computeFreatures(true);
		}
	}

public:
	int objectId;
	string halId;
	vector<int> activeHands;
	set<int> interactingObj;
	string affordance;
	pcl::PointXYZ startPoint;
	pcl::PointXYZRGBScore endPoint;
	float priorscore;
	float halscore;
	float predscore;
	float matchscore;
	float matchdist;
	int length;
	vector<PointXYZ> trajPoints;
	Frame startFrame;
	TransformG globalTransform;
	vector<Frame> segmentHal;
	string aid;
	string predlabel;
	string activity;

	hallucination(int objId, string act, string aff, pcl::PointXYZ sp, pcl::PointXYZRGBScore ep, int nf, vector<PointXYZ> traj, Frame &f,  TransformG gt, float score, string id ){
		objectId = objId;
		activity = act;
		affordance = aff;
		startPoint = sp;
		endPoint = ep;
		length = nf;
		startFrame = f;
		globalTransform = gt;
		aid = startFrame.sequenceId;
		priorscore = 0;
		matchdist = score;
		halscore = 0;
		predscore = 0;
		matchscore = 0;
		halId = id;
		trajPoints = traj;
		predlabel = "";
		initializeMaps();

	}
	hallucination(int objId, string act, string aff, pcl::PointXYZ sp, pcl::PointXYZRGBScore ep, int nf, vector<PointXYZ> traj, Frame &f,  TransformG gt, float score, string id, vector<int> &activehands ){
		objectId = objId;
		activity = act;
		affordance = aff;
		startPoint = sp;
		endPoint = ep;
		length = nf;
		startFrame = f;
		globalTransform = gt;
		aid = startFrame.sequenceId;
		priorscore = 0;
		matchdist = score;
		halscore = 0;
		predscore = 0;
		matchscore = 0;
		halId = id;
		activeHands = activehands;
		trajPoints = traj;
		predlabel = "";
		initializeMaps();

	}

	hallucination(int objId, string act, string aff, pcl::PointXYZ sp, pcl::PointXYZRGBScore ep, int nf, vector<PointXYZ> traj, Frame &f,  TransformG gt, float score, string id, vector<int> &activehands, set<int> interactingobj ){
		objectId = objId;
		activity = act;
		affordance = aff;
		startPoint = sp;
		endPoint = ep;
		length = nf;
		startFrame = f;
		globalTransform = gt;
		aid = startFrame.sequenceId;
		priorscore = 0;
		matchdist = score;
		halscore = 0;
		predscore = 0;
		matchscore = 0;
		halId = id;
		activeHands = activehands;
		interactingObj = interactingobj;
		trajPoints = traj;
		predlabel = "";
		initializeMaps();

	}
	hallucination(int objId, string act, string aff, pcl::PointXYZ sp, pcl::PointXYZRGBScore ep, int nf, vector<PointXYZ> traj, Frame &f,  TransformG gt, float score, string id,  set<int> interactingobj ){
		objectId = objId;
		activity = act;
		affordance = aff;
		startPoint = sp;
		endPoint = ep;
		length = nf;
		startFrame = f;
		globalTransform = gt;
		aid = startFrame.sequenceId;
		priorscore = 0;
		matchdist = score;
		halscore = 0;
		predscore = 0;
		matchscore = 0;
		halId = id;

		interactingObj = interactingobj;
		trajPoints = traj;
		predlabel = "";
		initializeMaps();

	}

	~hallucination(){
		segmentHal.clear();

	}

	float gethalscore(){
		return halscore;
	}
	float getpredscore(){
		return predscore;
	}
	void sethalscore(float score){
		cout << "Setting hal score of HAL:" << halId <<" to :" << score << endl;
		halscore = score;
	}
	void setpredscore(float score){
		cout << "Setting prediction score of HAL:" << halId <<" to :" << score << endl;
		predscore = score;
	}
	void setpredlabel(string l){
		predlabel = l;
	}

	void computeFrameFeatures (){
		//if(objectId >= 0) {
			createSegment();
			computeFeatures();
			string cmd = "cat orig_"+aid+"_data_obj_feats.txt hallucination_"+aid+"_"+halId+"_data_obj_feats.txt > hal_"+aid+"_data_obj_feats.txt";
			system(cmd.c_str());
			cmd = "cat orig_"+aid+"_data_skel_feats.txt hallucination_"+aid+"_"+halId+"_data_skel_feats.txt > hal_"+aid+"_data_skel_feats.txt";
			system(cmd.c_str());
			cmd = "cat orig_"+aid+"_data_skel_obj_feats.txt hallucination_"+aid+"_"+halId+"_data_skel_obj_feats.txt > hal_"+aid+"_data_skel_obj_feats.txt";
			system(cmd.c_str());
			cmd = "cat orig_"+aid+"_data_obj_obj_feats.txt hallucination_"+aid+"_"+halId+"_data_obj_obj_feats.txt > hal_"+aid+"_data_obj_obj_feats.txt";
			system(cmd.c_str());
			cmd = "cat orig_"+aid+"_data_temporal_obj_feats.txt hallucination_"+aid+"_"+halId+"_data_temporal_obj_feats.txt > hal_"+aid+"_data_temporal_obj_feats.txt";
			system(cmd.c_str());
			cmd = "cat orig_"+aid+"_data_temporal_skel_feats.txt hallucination_"+aid+"_"+halId+"_data_temporal_skel_feats.txt > hal_"+aid+"_data_temporal_skel_feats.txt";
			system(cmd.c_str());
		//}
	}
	void setId(string id){
		halId = id;
	}

	string getSegmentLabelString(){
		string label;
		label = activity;
		int numObj = startFrame.objects.size();
		for(int n = 0; n <numObj ; n++){
			if(n != objectId && interactingObj.find(n) == interactingObj.end()){
				label += ":stationary" ;
			} else if(interactingObj.find(n) != interactingObj.end()){
				label += ":"  + interactaffMap[activity] ;
			}
			else{
				label += ":" + actaffMap[activity] ;
			}
		}
		return label;
	}

	void writeLabelFile(map<int, map<int, string> >  &PredLabelMap,  map<int, string >  &PredActLabelMap, map<int, map<string, int> >  &FrameMap){
		std::ofstream outFile;
		string filename = "hal_labeling_"+ startFrame.sequenceId +".txt";
		outFile.open(filename.c_str()); //, ios::app);
		int numObj = PredLabelMap[1].size();
		for(map<int, string >::iterator it = PredActLabelMap.begin() ; it != PredActLabelMap.end(); it++ ){
			int segNum = it->first;
			outFile << aid << "," << FrameMap[segNum]["sf"] << "," << FrameMap[segNum]["ef"] << "," << it->second  ;

			for(int n = 1; n <=numObj ; n++){
				outFile  << "," << PredLabelMap[segNum][n];
			}
			outFile << endl;
		}
		outFile << aid << "," << sf << "," << ef << "," << activity  ;
		for(int n = 0; n <numObj ; n++){
			if(n != objectId && interactingObj.find(n) == interactingObj.end()){
				outFile << "," << "stationary" ;
			} else if(interactingObj.find(n) != interactingObj.end()){
				outFile << ","  << interactaffMap[activity] ;
			}
			else{
				outFile << ","  << actaffMap[activity] ;
			}
		}
		outFile << endl;
		outFile.close();
	}

	void writeTrajectoryFile(){
		std::ofstream outFile;
		string filename = "hal_"+startFrame.sequenceId+"_traj.txt";
		outFile.open(filename.c_str(), ios::app);
		string label = getSegmentLabelString();
		int numObj = startFrame.objects.size();
		outFile << startFrame.sequenceId << "," << objectId << "," << halId << "," << activity << "," << halscore << "," << trajPoints.size() << "," << numObj << "," << label << "," << predscore << "," << predlabel << endl;
		for(int i = 0; i < trajPoints.size(); i ++){
			outFile << trajPoints.at(i).x << "," << trajPoints.at(i).y << "," << trajPoints.at(i).z << endl;
		}
		//outFile << endl;
		outFile.close();
	}
	double getDistanceSqrBwPoints(pcl::PointXYZ &p1, pcl::PointXYZ &p2) {
	    double dist = 0;
	    dist = pow((p1.x - p2.x), 2);
	    dist += pow((p1.y - p2.y), 2);
	    dist += pow((p1.z - p2.z), 2);
	    //  dist = sqrt(dist);
	    return dist;
	}

	double distanceFromTrajectory(vector<pcl::PointXYZ> &gt){
		double dist =0;
		for(size_t i =0; i < gt.size(); i ++){
			double mindist = 100000000;
			for(size_t j =0; j < trajPoints.size(); j ++){
				double dist = getDistanceSqrBwPoints(gt.at(i), trajPoints.at(j));
				if(dist < mindist){mindist = dist;}
			}
			dist += sqrt(mindist);
		}
		return dist;
	}


	double findTrajectoryMatch(vector<Frame> &allFrames, int len){
		// get the corresponding trajectory from the gt
		int numF = allFrames.size();
		vector<pcl::PointXYZ> gtTraj;
		int start = numF-len;
		for(size_t i = start; i < numF; i++){
			if(objectId>=0){
				gtTraj.push_back(allFrames.at(i).objects.at(objectId).getCentroid());
			}else{
				int numJoint = allFrames.at(i).skeleton.transformed_joints.size();
				gtTraj.push_back(allFrames.at(i).skeleton.transformed_joints.at(numJoint+objectId));
			}
		}
		// distance moved by the joint/object
		double dist = 0;
		if( gtTraj.size() != 0 ){
			dist = sqrt(getDistanceSqrBwPoints(gtTraj.at(0), gtTraj.at(gtTraj.size()-1)));
		}
		// smooth the trajectory (apply median filter)

		//
		gtTraj.clear();
		start = numF-len+(length - trajPoints.size());
		for(size_t i = start; i < numF; i++){
			if(objectId>=0){
				gtTraj.push_back(allFrames.at(i).objects.at(objectId).getCentroid());
			}else{
				int numJoint = allFrames.at(i).skeleton.transformed_joints.size();
				gtTraj.push_back(allFrames.at(i).skeleton.transformed_joints.at(numJoint+objectId));
			}
		}
		// find the match score
		matchdist += distanceFromTrajectory(gtTraj);
		cout << "matchScore: "<< dist << "," << matchdist << "," << matchscore << endl;
		if(len !=0){
			matchscore =  (1/(1+exp(((matchdist/len)-150)/20)))*(1/(1+exp((-dist+15)/5)));
		}else if(activity.compare("moving") == 0 || activity.compare("opening") == 0 ||activity.compare("closing") == 0 || activity.compare("cleaning") == 0 || activity.compare("reaching") == 0){
		    matchscore =  (1/(1+exp((matchdist-150)/20)))*(1/(1+exp((-dist+15)/5)));
		}else{
			matchscore =  1/(1+exp((matchdist-150)/20));
		}

		return matchscore;
	}
	bool isValidTrajectory(){
		// TODO average distance between points set to 90
		float avgDist=0;
		for(size_t i = 1; i < trajPoints.size(); i ++){
			avgDist += sqrt(getDistanceSqrBwPoints(trajPoints.at(i), trajPoints.at(i-1)));
		}
		avgDist = avgDist/(trajPoints.size()-1);
		if(avgDist < 200){
			saveTrajectory();
			return true;
		}
		return false;
	}

};

class affordanceMaps {

private:
	TransformG globalTransform;
	map< string, set<string> > AffMap;
	double getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZ &p2);
	double getDistanceSqrBwPoints(pcl::PointXYZ &p1, pcl::PointXYZRGBScore &p2);
	double getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZRGBScore &p2);
	double getDistanceSqrBwPoints(pcl::PointXYZ &p1, pcl::PointXYZ p2);

	float getMinDistance(Frame &frame, int joint, int objId);
	int getObjId(Frame &frame, PointXYZRGBScore &point);
	pcl::PointXYZRGBScore getImagePoint(pcl::PointXYZRGBScore  p,TransformG globalTransform );
	void saveFloatImage ( const char* filename, const IplImage * image );
	void saveImage(pcl::PointCloud<PointT> & cloud, string imagename) ;
	double getPdf_VM(float alpha, float mean, float kappa);
	void drinkabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples  );
	void cleanabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, int objRef );
	void placeabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, pcl::PointXYZ refPoint);
	void closeabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples, int objId );
	void openabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples , int objId, int joint );

	//void getSamplePoints(pcl::PointXYZ p, float size, float sr, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud );
	void getObjectPoints(Frame &f, int objid, pcl::PointCloud<pcl::PointXYZRGBScore> &cloud );
	void writeHeatMap(string filename, vector<pcl::PointXYZRGBScore> &imageScorePoints);
	vector<pcl::PointXYZRGBScore> generateSamples(pcl::PointCloud<pcl::PointXYZRGBScore> &cloud);
	vector<pcl::PointXYZRGBScore> generateReachSamples(pcl::PointCloud<pcl::PointXYZRGBScore> &cloud);
	pair <float,int> getMinObjDistance(Frame &frame, int joint);
	vector<pcl::PointXYZRGBScore> generatePourabilityHeatMap(Frame &f, int s, int u, int pourableObj,int pourtoObj);
	void pourabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &samples ,int obj, int objRef );
	std::set<string> getAffordanceList(string label);
	void readAffMap();
	void reachabilityScores(Frame &frame, int s, int q, pcl::PointCloud<pcl::PointXYZRGBScore> &outcloud, int handJoint);
	vector<pcl::PointXYZRGBScore> generateReachabilityHeatMap(Frame &f, int s, int u, int handJoint );
	vector<pcl::PointXYZRGBScore> generateDrinkabilityHeatMap(Frame &f, int s, int u );
	vector<pcl::PointXYZRGBScore> generateCleanabilityHeatMap(Frame &f, int s, int u, int cleanerObj,int cleanableObj);
	vector<pcl::PointXYZRGBScore> generatePlaceabilityHeatMap(Frame &f, int s, int u, int placeableObj,pcl::PointXYZ refLoc );
	vector<pcl::PointXYZRGBScore> generateOpenabilityHeatMap(Frame &f, int s, int u, int openableObj);
	vector<pcl::PointXYZRGBScore> generateCloseabilityHeatMap(Frame &f, int s, int u, int closeableObj );
	void updateH(vector<Frame> &segment,  hallucination  &hal, map< int, vector < hallucination > > &hallucinations, int segNum, int segLen, int numframeseen, int updateC,  map< pair < int ,pair< string, string> > , int> &particleCounts);

public:
    
    //pcl::PointCloud<PointT> origCloud;
    //pcl::PointCloud<PointT> sampleCloud;



    void generateHallucinations(vector<Frame> &segment, map < int, vector <hallucination > > &hallucinations, int segNum, int segLen, int numframeseen ,int updateC);
    void updateHallucinations(vector<Frame> &segment, map < int, vector <hallucination > > &hallucinations, int segNum, int segLen, int numf, int updateC);

    affordanceMaps(TransformG &gT);

    ~affordanceMaps();
};

/** 
there are 11 joints that have both orientation (3x3) and position (x,y,z) data
        XN_SKEL_HEAD,
        XN_SKEL_NECK,
        XN_SKEL_TORSO,
        XN_SKEL_LEFT_SHOULDER,
        XN_SKEL_LEFT_ELBOW,
        XN_SKEL_RIGHT_SHOULDER,
        XN_SKEL_RIGHT_ELBOW,
        XN_SKEL_LEFT_HIP,
        XN_SKEL_LEFT_KNEE,
        XN_SKEL_RIGHT_HIP,
        XN_SKEL_RIGHT_KNEE
	
there are 4 joints that have only position (x,y,z) data
        XN_SKEL_LEFT_HAND,
        XN_SKEL_RIGHT_HAND,
        XN_SKEL_LEFT_FOOT,
        XN_SKEL_RIGHT_FOOT

 data[][0~8]    -> orientation (3x3 matrix)
                     3x3 matrix is stored as 
                        0 1 2
                        3 4 5
                        6 7 8
                     read PDF for description about 3x3 matrix 
 data[][9~11]   -> x,y,z position for eleven joints
 
 data_CONF[][0]   -> confidence value of orientation  (data[][0~8]) 
 data_CONF[][1]   -> confidence value of xyz position (data[][9~11])
 
 data_pos[][0~2] -> x,y,z position for four joints
 data_pos_CONF[]  -> confidence value of xyz position (data_pos[][0~2])
 
X_RES and Y_RES are in constants.h, so just use them.
 IMAGE[X_RES][Y_RES][0~2]   -> RGB values
 IMAGE[X_RES][Y_RES][3]     -> depth values

 
 */
