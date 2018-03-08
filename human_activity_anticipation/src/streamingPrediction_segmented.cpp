/* 
 * File:   featureGeneration.cpp
 * Author: hema
 *
 * Created on August 29, 2011, 6:16 PM
 */

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

//#include "Point2D.h"
//#include "HOGFeaturesOfBlock.h"
//#include "HOG.h"
#include <pcl/point_types.h>
typedef pcl::PointXYZRGB PointT;
#include "includes/point_types.h"
#include "includes/CombineUtils.h"

#include "readData.cpp"
#include "frame.cpp"
//#include "generateTrajectories.cpp"
#include "frameFeatures.cpp"
#include "svm_struct/svm_struct_classify_stream.h"
#include "affordanceMaps.cpp"
//#include "features_multiFrame.cpp"
#include <boost/regex.hpp>
#include <boost/algorithm/string/regex.hpp>

using namespace std;


bool USE_HOG = false;

bool useHead = true;
bool useTorso = true;
bool useLeftArm = true;
bool useRightArm = true;
bool useLeftHand = true;
bool useRightHand = true;
bool useFullBody = true;
bool useImage = true;
bool useDepth = true;
bool useSkeleton = true;

map<string, string> classMap;
map<string, string> affMap;

map<string, string> data_act_map;
map<string, vector<string> > data_obj_map;
map<string, vector<string> > data_obj_type_map;
map<string, set<int> > FrameList;
map<string, map<int, set<int> > > SegmentList;
map<string, map<int, int> > SegmentFrameCount;
map<string, map<int, map<int, string> > > LabelMap;
map<string, map<int, map<int, string> > > PredLabelMap;
map<string, map<int, string > > ActLabelMap;
map<string, map<int, string > > PredActLabelMap;
map<string, map<int, map<string, int> >  >  FrameMap;
string dataLocation;

void initializeMaps(){
	classMap["1"] = "reaching";
	classMap["2"] = "moving";
	classMap["3"] = "pouring";
	classMap["4"] = "eating";
	classMap["5"] = "drinking";
	classMap["6"] = "opening";
	classMap["7"] = "placing";
	classMap["8"] = "closing";
	classMap["9"] = "null";
	classMap["10"] = "cleaning";

	affMap["1"] =  "movable";
	affMap["2"] =  "stationary";
	affMap["3"] =  "reachable";
	affMap["4"] =  "pourable";
	affMap["5"] =  "pourto";
	affMap["6"] =  "containable";
	affMap["7"] =  "drinkable";
	affMap["8"] =  "openable";
	affMap["9"] =  "placeable";
	affMap["10"] =  "closeable";
	affMap["11"] =  "cleanable";
	affMap["12"] =  "cleaner";
}

// print error message

void errorMsg(string message) {
	cout << "ERROR! " << message << endl;
	exit(1);
}

void parseChk(bool chk) {
	if (!chk) {
		errorMsg("parsing error.");
	}
}

void readLabelFile() {
    const string labelfile = dataLocation + "labeling_ijrr_sampled.txt";

    ifstream file((char*) labelfile.c_str(), ifstream::in);
    map<string, int> SegCount;
    string line;
    int count = 0;
    while (getline(file, line)) {
        stringstream lineStream(line);
        string element1, element2;
        parseChk(getline(lineStream, element1, ','));

        if (element1.compare("END") == 0) {
            break;
        }

        if (SegCount.count(element1) == 0) {
           SegCount[element1] = 1;
        } else{
        	SegCount[element1] += 1;
        }
        parseChk(getline(lineStream, element2, ',')); // get start frame
		int startF = atoi(element2.c_str());
		parseChk(getline(lineStream, element2, ',')); // get end frame
		int endF = atoi(element2.c_str());
        parseChk(getline(lineStream, element2, ',')); // get activity
        ActLabelMap[element1][SegCount[element1]] = element2.c_str();
        int objId = 1;
        while (getline(lineStream, element2, ',')) {
            LabelMap[element1][SegCount[element1]][objId] =element2.c_str();
            int seg = SegCount[element1];
            //cout << element1 << "," << seg <<"," << objId << "," << LabelMap[element1][seg][objId] << endl;
            objId ++;
            FrameMap[element1][SegCount[element1]]["sf"] = startF;
            FrameMap[element1][SegCount[element1]]["ef"] = endF;
        }

     //   cout << "\t" << element1 << " -> \"" << data_act_map[element1] << "\"" << endl;
        count++;
    }
    file.close();
}

void readSegmentsFile() {
	//const string labelfile = dataLocation + "Segmentation.txt";
	const string labelfile = dataLocation + "Segmentation_ijrr.txt";

	ifstream file((char*) labelfile.c_str(), ifstream::in);

	string line;
	int count = 0;
	while (getline(file, line)) {
		stringstream lineStream(line);
		string element1, element2;
		parseChk(getline(lineStream, element1, ';'));

		if (element1.compare("END") == 0) {
			break;
		}

		//parseChk(getline(lineStream, element2, ',')); // get actor
		while (getline(lineStream, element2, ';')) {
			int pos = element2.find_first_of(':');
			int cluster = atoi(element2.substr(0, pos).c_str());
			SegmentFrameCount[element1][cluster] = 0;
			cout << "cluster: " << cluster << " :";
			//string start = element2.substr(0, pos);
			string rest = element2.substr(pos + 1);
			pos = rest.find_first_of(',');
			while (pos != string::npos) {

				int fnum = atoi(rest.substr(0, pos).c_str());
				cout << fnum << ",";
				SegmentList[element1][cluster].insert(fnum);
				SegmentFrameCount[element1][cluster] += 1;
				rest = rest.substr(pos + 1);
				pos = rest.find_first_of(',');
			}
			int fnum = atoi(rest.substr(0, pos).c_str());
			cout << fnum << ",";
			SegmentList[element1][cluster].insert(fnum);
			SegmentFrameCount[element1][cluster] += 1;
			cout << endl;

		}

		cout << "\t" << element1 << endl;
		count++;
	}
	file.close();
}

// read label file to get the frame list for each activity
/*
void readLabelFile() {
	const string labelfile = dataLocation + "labeledFrames.txt";

	ifstream file((char*) labelfile.c_str(), ifstream::in);

	string line;
	int count = 0;
	while (getline(file, line)) {
		stringstream lineStream(line);
		string element1, element2;
		parseChk(getline(lineStream, element1, ','));

		if (element1.compare("END") == 0) {
			break;
		}

		//parseChk(getline(lineStream, element2, ',')); // get actor
		while (getline(lineStream, element2, ',')) {
			FrameList[element1].insert(atoi(element2.c_str()));
		}

		//   cout << "\t" << element1 << " -> \"" << data_act_map[element1] << "\"" << endl;
		count++;
	}
	file.close();
}*/

// read file that maps data and activity

void readDataActMap(string actfile) {
	const string mapfile = dataLocation + actfile;

	printf("Opening map of data to activity: \"%s\"\n",
			(char*) mapfile.c_str());
	ifstream file((char*) mapfile.c_str(), ifstream::in);

	string line;
	int count = 0;
	while (getline(file, line)) {
		stringstream lineStream(line);
		string element1, element2, element3;
		parseChk(getline(lineStream, element1, ','));

		if (element1.compare("END") == 0) {
			break;
		}
		parseChk(getline(lineStream, element2, ','));
		if (element1.length() != 10) {
			errorMsg("Data Act Map file format mismatch..");
		}

		data_act_map[element1] = element2;
		parseChk(getline(lineStream, element3, ',')); // get actor
		while (getline(lineStream, element3, ',')) {
                        cout << element3 << endl;
			//vector<string> fields;
			//boost::split_regex( fields, element3, boost::regex( ":" ) );
                        int v = element3.find(":",0);
			cout << element3.substr(0,v) << endl;
			cout << element3.substr(v+1) << endl;

			data_obj_map[element1].push_back(element3.substr(0,v));
                        
			data_obj_type_map[element1].push_back(element3.substr(v+1));
		}

		cout << "\t" << element1 << " : " << data_act_map[element1] << endl;
		count++;
	}
	file.close();

	if (count == 0) {
		errorMsg("File does not exist or is empty!\n");
	}
	printf("\tcount = %d\n\n", count);
}

void printData(FILE* pRecFile, vector<double> feats) {
	for (int i = 0; i < feats.size(); i++) {
		if (feats.at(i) == 0)
			fprintf(pRecFile, "%.1f,", feats.at(i));
		else
			fprintf(pRecFile, "%.7f,", feats.at(i));
	}
}

void printData(FILE* pRecFile, double * feats, int numFeats) {
	for (int i = 0; i < numFeats; i++) {
		fprintf(pRecFile, "%.7f,", feats[i]);
	}
}

void printData(FILE* pRecFile, int * feats, int numFeats) {
	for (int i = 0; i < numFeats; i++) {
		fprintf(pRecFile, "%d,", feats[i]);
	}
}

int getCluster(int frameNum, string id) {
	for (map<int, set<int> >::iterator i = SegmentList[id].begin();
			i != SegmentList[id].end(); i++) {
		if (i->second.find(frameNum) != i->second.end()) {
			return i->first;
		}
	}
	return 0;
}

vector<string> getObjTypes(){

}

float getScore(string filename){
	ifstream file((char*) filename.c_str(), ifstream::in);
	string line;
	getline(file, line);
	return atof(line.c_str());

}

string interpretPrediction(string filename, string aid, bool orig){
	string lastseglabel;
		vector<pair<string, string> > predLabeling;
		ifstream file((char*) filename.c_str(), ifstream::in);

		string line;
		int count = 0;
		while (getline(file, line)) {
			int segCount = 0;
			stringstream lineStream(line);
			string element1, element2, element3, element4;
			while(getline(lineStream, element1, ';')){
				segCount ++;
			//while(element1.compare("")!=0){
				stringstream segStream(element1);

				while(getline(segStream, element2, ',')){
					//cout << "labeling: "  << element2 << endl;
					stringstream nodeStream(element2);
					getline(nodeStream, element3, ':');
					getline(nodeStream, element4, ':');
					predLabeling.push_back(make_pair(element3, element4));

				}
				int last = predLabeling.size() -1;
				cout << "activity: " << classMap[predLabeling.at(last).second] << endl;
				lastseglabel = classMap[predLabeling.at(last).second];
				if (orig) {PredActLabelMap[aid][segCount]= classMap[predLabeling.at(last).second];}
				map<int,string> tmpmap;
				for (int c = 0 ; c < last; c++){
					cout << "affordance :: obj id:" << predLabeling.at(c).first << " label: " << affMap[predLabeling.at(c).second] << endl;
					tmpmap[atoi(predLabeling.at(c).first.c_str())] = affMap[predLabeling.at(c).second];
					if (orig) {PredLabelMap[aid][segCount][atoi(predLabeling.at(c).first.c_str())] = affMap[predLabeling.at(c).second];}
				}
				for(map<int,string>::iterator it=tmpmap.begin(); it!=tmpmap.end(); ++it){
					lastseglabel += ":" + it->second ;
				}
				predLabeling.clear();
				//getline(lineStream, element1, ';');
				cout << "element1:" << element1 << ":" << endl;

			}

		}
		file.close();
		return lastseglabel;



}

double getDistanceSqrBwPoints(pcl::PointXYZRGB &p1, pcl::PointXYZ &p2) {
    double dist = 0;
    dist = pow((p1.x - p2.x), 2);
    dist += pow((p1.y - p2.y), 2);
    dist += pow((p1.z - p2.z), 2);
    //  dist = sqrt(dist);
    return dist;
}
pair <float,int> getMinObjDistance(Frame &frame, int joint){

	float minDist = 1000000;
	int objIdx = -1;
	for( int i = 0; i < frame.objects.size(); i ++){

			for(int j = 0; j < frame.objects.at(i).pcInds.size(); j++){
				int index = frame.objects.at(i).pcInds.at(j);
				float dist = sqrt(getDistanceSqrBwPoints(frame.cloud.at(index),frame.skeleton.transformed_joints.at(joint)));
				if(dist<minDist){minDist = dist; objIdx = i;}
			}
	}
	return make_pair(minDist, objIdx);
}

void writeGTtrajectories_(vector<Frame> &activityFrames,int segmentSize, int segNum){

	std::ofstream outFile;
	outFile.open("gt_traj.txt", ios::app);
	float activeObjThreshold = 100;
	int frameIdx = activityFrames.size() - segmentSize;
	int numJoints = activityFrames.at(frameIdx).skeleton.transformed_joints.size();
	// find active object
	int leftHandJoint =  numJoints -2;
	int rightHandJoint = numJoints -1;
	pair <float , int> leftDist = getMinObjDistance(activityFrames.at(frameIdx),leftHandJoint);
	pair <float , int> rightDist = getMinObjDistance(activityFrames.at(frameIdx),rightHandJoint);
	// find active object
	for(int j = 0; j < activityFrames.at(frameIdx).objects.size(); j++){
		// check if active
		cout << "object id : " << j << " " ;
		bool active = false;
		int hand = 0;
		if (leftDist.first < activeObjThreshold && leftDist.second == j ){
			active = true;
			hand = leftHandJoint;
			cout << "left hand active ";
		}else if (rightDist.first < activeObjThreshold && rightDist.second ==j ) {
			active = true;
			hand = rightHandJoint;
			cout << "right hand active " ;
		}else {
			// object cannot moving.. no trajectory generated
			cout << "not in contact " ;
		}
		if (active){
			string activity =  ActLabelMap[activityFrames.at(frameIdx).sequenceId][segNum];
			outFile << activityFrames.at(frameIdx).sequenceId << ","  << segNum << "," << j << "," << activity << "," << segmentSize << endl;
			for(int i = frameIdx; i < activityFrames.size(); i ++){
				outFile << activityFrames.at(i).objects.at(j).getCentroid().x << "," << activityFrames.at(i).objects.at(j).getCentroid().y << "," << activityFrames.at(i).objects.at(j).getCentroid().z << endl;
			}


		}
	}
	outFile.close();

}

void writeGTtrajectories(vector<Frame> &activityFrames,int segmentSize, int segNum){
	int frameIdx = activityFrames.size() - segmentSize;
	std::ofstream outFile;
	string filename = "gt_"+activityFrames.at(frameIdx).sequenceId+"_traj.txt";
	outFile.open(filename.c_str(), ios::app);

	string activity =  ActLabelMap[activityFrames.at(frameIdx).sequenceId][segNum];
	if(activity.compare("moving") == 0){

	for(int j = 0; j < activityFrames.at(frameIdx).objects.size(); j++){
		// check if moving object

		if (LabelMap[activityFrames.at(frameIdx).sequenceId][segNum][j+1].compare("movable") == 0){
			outFile << activityFrames.at(frameIdx).sequenceId << ","  << segNum << "," << j << "," << activity << "," << segmentSize << endl;
			for(int i = frameIdx; i < activityFrames.size(); i ++){
				outFile << activityFrames.at(i).objects.at(j).getCentroid().x << "," << activityFrames.at(i).objects.at(j).getCentroid().y << "," << activityFrames.at(i).objects.at(j).getCentroid().z << endl;
			}
		}
	}
	}
	outFile.close();

}

void writeTestfile(string aid){
	std::ofstream outFile;
	string filename = "test_"+aid+".txt";
	outFile.open(filename.c_str());
	outFile << aid << endl;
	outFile.close();
}

std::vector<char *>  gethalargs(string aid, string halid, string fold){
	  string cmd_hal = "a --m svmstruct_mrf_act_dyn --sf false --temporal true --hal true --fold "+fold+" test_"+ aid +".txt fold"+ fold+ "/model.txt pred_"+aid +"_" + halid + ".txt";
	  //string cmd_hal = "a --m svmstruct_mrf_act_dyn --sf false --temporal true --hal true test.txt fold"+ fold+ "/model.txt pred.txt";

	  std::vector<char *> args_hal;
	  std::istringstream iss_hal(cmd_hal);
	  std::string token;
	  while (iss_hal >> token) {
		  char *arg_hal = new char[token.size() + 1];
		  copy(token.begin(), token.end(), arg_hal);
		  arg_hal[token.size()] = '\0';
		  args_hal.push_back(arg_hal);
	  }
	  args_hal.push_back(0);
	  return args_hal;
}

/*
 * 
 */
int main(int argc, char** argv) {

	dataLocation = (string) argv[1] + "/";
	string actfile = (string) argv[2];
	string mirrored_dataLocation = "";
	string fold = (string) argv[3];
//    string outputFile = "data_extracted/features.txt"; //+ (string)argv[1] + ".txt";

	readDataActMap(actfile);
	readLabelFile();
	readSegmentsFile();
	initializeMaps();
	int  INITIALFRAMECOUNT = 20;
	int STEPSIZE = 10;

	// get all names of file from the map
	vector<string> all_files;
	map<string, string>::iterator it = data_act_map.begin();
	while (it != data_act_map.end()) {
		all_files.push_back(it->first);
		cout << it->first << endl;
		it++;
	}
	printf("Number of Files to be processed = %d\n", all_files.size());
//    printf("Processed data goes to (%s)\n\n", (char*) outputFile.c_str());

	//FILE* pRecFile;
	//pRecFile = fopen((char*) outputFile.c_str(), "w");

	double **data; //[JOINT_NUM][JOINT_DATA_NUM];
	int **data_CONF; //[JOINT_NUM][JOINT_DATA_TYPE_NUM]
	double **pos_data; //[POS_JOINT_NUM][POS_JOINT_DATA_NUM];
	int *pos_data_CONF; //[POS_JOINT_NUM]
	data = new double*[JOINT_NUM];
	data_CONF = new int*[JOINT_NUM];
	for (int i = 0; i < JOINT_NUM; i++) {
		data[i] = new double[JOINT_DATA_NUM];
		data_CONF[i] = new int[JOINT_DATA_TYPE_NUM];
	}
	pos_data = new double*[POS_JOINT_NUM];
	pos_data_CONF = new int[POS_JOINT_NUM];
	for (int i = 0; i < POS_JOINT_NUM; i++) {
		pos_data[i] = new double[POS_JOINT_DATA_NUM];
	}

	int ***IMAGE; // [X_RES][Y_RES]
	IMAGE = new int**[X_RES];
	for (int i = 0; i < X_RES; i++) {
		IMAGE[i] = new int*[Y_RES];
		for (int j = 0; j < Y_RES; j++) {
			IMAGE[i][j] = new int[RGBD_data];
		}
	}

	//fileList[1] = "data/0829113826_obj_2.txt";
	vector<vector<double> > objData;
	vector<vector<int> > objPCInds;
	string lastActId = "0";
	for (size_t i = 0; i < all_files.size(); i++) {
		writeTestfile(all_files.at(i));
		int count = 1;
		bool hallucinate = true;
		vector<string> fileList(data_obj_map[all_files.at(i)].size());
		for (size_t j = 0; j < data_obj_map[all_files.at(i)].size(); j++) {
			fileList.at(j) = dataLocation + "/" + all_files.at(i) + "_obj"
					+ data_obj_map[all_files.at(i)].at(j) + ".txt";
			//fileList.at(j) = dataLocation + "/new_object_features/" + all_files.at(i) + "_object_features_" + data_obj_map[all_files.at(i)].at(j) + ".txt";
		}
		vector<string> objPCFileList(data_obj_map[all_files.at(i)].size());
		for (size_t j = 0; j < data_obj_map[all_files.at(i)].size(); j++) {
			objPCFileList.at(j) = dataLocation + "/objects/" + all_files.at(i)
					+ "_obj" + data_obj_map[all_files.at(i)].at(j) + ".txt";
			//objPCFileList.at(j) = dataLocation + "/tracking-objects/" + all_files.at(i) + "_object_" + data_obj_map[all_files.at(i)].at(j) + ".txt";
		}

		// for both mirrored and non mirrored data make j<2 ; for now use only mirrored

		Frame::FrameNum = 0;
		bool mirrored = false;
		bool skipOdd = false;
		string transformfile = dataLocation + all_files[i]
				+ "_globalTransform.txt";

		TransformG globalTransform = readTranform(transformfile);
		globalTransform = globalTransform.inverse();

		readData* DATA = new readData(dataLocation, all_files[i], data_act_map,
				i + 1, mirrored, mirrored_dataLocation, skipOdd, fileList,
				objPCFileList);
		int status = 0;
		//generateTrajectories gt;
		FrameFeatures ff(true, "orig_"+all_files[i]);
		//vector<FrameFeatures *> ffh;
		//for (int h = 0; h < 3; h++) {
		//	string name = "hal_" + boost::lexical_cast<string>(h);

		//	ffh.push_back(new FrameFeatures(true, name));
		//}

		vector<Frame> segmentHal;
		vector<Frame> activityFrames;
		affordanceMaps amaps(globalTransform);
		/*FeaturesSkel* features_skeleton;
		 if (useSkeleton)
		 features_skeleton = new FeaturesSkel((char*) all_files[i].c_str(), pRecFile, mirrored);


		 FeaturesSkelRGBD* features_rgbd = new FeaturesSkelRGBD(pRecFile, mirrored);
		 */
		int frameNum  = 0;
		int oldSegNum = 1;
		// read first n frames
		int segCount = 1;
		INITIALFRAMECOUNT = SegmentFrameCount[all_files[i]][segCount];
		for(int numf = 0; numf < INITIALFRAMECOUNT ; numf ++ ){
			// if status +1 is labeled then read frame else skip frame
			int segNum =  getCluster(status+1, all_files[i]);
			if (segNum == 0) { numf -- ; status = DATA->skipNextFrame(); cout << "skipped frame:" << status << endl; continue;}
			status = DATA->readNextFrame(data, pos_data, data_CONF,
																pos_data_CONF, IMAGE, objData, objPCInds);
			if (status > 0) {
				frameNum ++;
				Frame frame(IMAGE, data, pos_data, objData, all_files[i],
						frameNum, transformfile, objPCInds, data_obj_type_map[all_files[i]]);
				cout << "read frame:"<<  status << endl;
				//compute the frame features
				ff.setCurrentFrame(frame);
				ff.computeFreatures(true);
				activityFrames.push_back(frame);

			}else{
				cout << "end of activity! Activity smaller than intialization size" << endl;
			}

		}
		
		// compute the labels for the activity so far
 		string cmd = "a --m svmstruct_mrf_act_dyn --sf false --temporal true --hal false --fold "+fold+" test_" + all_files.at(i)+".txt fold"+ fold + "/model.txt pred_"+all_files[i]+".txt";
 		//string cmd = "a --m svmstruct_mrf_act_dyn --sf false --temporal true --hal false test.txt fold"+ fold + "/model.txt pred.txt";

		std::vector<char *> args;
		std::istringstream iss(cmd);

		std::string token;
		while (iss >> token) {
			char *arg = new char[token.size() + 1];
			copy(token.begin(), token.end(), arg);
			arg[token.size()] = '\0';
			args.push_back(arg);
		}
		args.push_back(0);



		svm_classifier_init(14 , &args[0]);
		svm_classifier( 14 , &args[0] );
		interpretPrediction("pred_"+all_files[i]+".txt", all_files[i], true);
			// compute segment features
			// get the labeling
		//map< int, vector<hallucination> > allHal;
		while (status > 0) {
			count ++;
			segCount ++;
			STEPSIZE = SegmentFrameCount[all_files[i]][segCount];
			map< int,vector < hallucination > > hallucinations;
			if(hallucinate){
				// generate affordance maps and sample end points

				amaps.generateHallucinations(activityFrames,hallucinations,segCount,STEPSIZE,0,0);
				for(size_t h = 0; h < hallucinations[0].size(); h ++ ){

					//if(hallucinations[0].at(h).objectId >= 0){
					hallucinations[0].at(h).computeFrameFeatures();
					hallucinations[0].at(h).writeLabelFile(PredLabelMap[all_files[i]],PredActLabelMap[all_files[i]], FrameMap[all_files[i]]);
					// compute the score
					cout << "calling classifier for hallucination :" << hallucinations[0].at(h).halId << endl;
					// evaluate the score function
					std::vector<char *> args_hal = gethalargs( all_files[i],  hallucinations[0].at(h).halId, fold);
		            svm_classifier( 14 , &args_hal[0] );
		            hallucinations[0].at(h).sethalscore(getScore("score_"+all_files[i]+".txt"));
		            hallucinations[0].at(h).setpredscore(getScore("predscore_"+all_files[i]+".txt"));
		            string predlabel = interpretPrediction("pred_"+all_files[i]+"_"+hallucinations[0].at(h).halId+".txt", all_files[i], false);
		            hallucinations[0].at(h).setpredlabel(predlabel);
		            hallucinations[0].at(h).writeTrajectoryFile();
		            //}
				}

			}

			// obtain the next set of frames

			if (STEPSIZE <= 2) {
				// last dummy segment... so exit
				break;
			}
			cout << "segment length of segment " << segCount << " is " << STEPSIZE << endl;
			//vector<Frame> segment;
			set<int> intermediateframes;
			for(int index =1; index<10; index++ ){
				intermediateframes.insert(int(index*0.1*STEPSIZE));
			}
			int updateCounter =0;
			for (int numf = 0; numf < STEPSIZE; numf++) {
				// if status +1 is labeled then read frame else skip frame
				 int segNum =  getCluster(status+1, all_files[i]);
				 if (segNum == 0) {
					 numf -- ;
					 status = DATA->skipNextFrame();
					 if(status > 0) continue;
					 break;
				 }
				status = DATA->readNextFrame(data, pos_data, data_CONF,
						pos_data_CONF, IMAGE, objData, objPCInds);
				frameNum ++;
				if (status > 0) {
					Frame frame(IMAGE, data, pos_data, objData, all_files[i],
							frameNum, transformfile, objPCInds, data_obj_type_map[all_files[i]] );

					//compute the frame features
					ff.setCurrentFrame(frame);
					ff.computeFreatures(true);
					activityFrames.push_back(frame);
				} else {
					cout
							<< "end of activity! Activity smaller than stepsize"
							<< endl;
					break;
				}
				if(intermediateframes.find(numf)!=intermediateframes.end()){
					// update hallucinations
					updateCounter++;
					amaps.updateHallucinations(activityFrames,hallucinations,segCount,STEPSIZE, numf, updateCounter);

					for(size_t h = 0; h < hallucinations[updateCounter].size(); h ++ ){

						if(hallucinations[updateCounter].at(h).gethalscore() == 0){
							hallucinations[updateCounter].at(h).computeFrameFeatures();
							hallucinations[updateCounter].at(h).writeLabelFile(PredLabelMap[all_files[i]],PredActLabelMap[all_files[i]], FrameMap[all_files[i]]);
							// compute the score
							cout << "calling classifier for hallucination :" << hallucinations[updateCounter].at(h).halId << endl;
							// evaluate the score function
							std::vector<char *> args_hal = gethalargs( all_files[i],  hallucinations[updateCounter].at(h).halId, fold);
							svm_classifier( 14 , &args_hal[0] );
							hallucinations[updateCounter].at(h).sethalscore(getScore("score_"+all_files[i]+".txt"));
							hallucinations[updateCounter].at(h).setpredscore(getScore("predscore_"+all_files[i]+".txt"));
							string predlabel = interpretPrediction("pred_"+all_files[i]+"_"+hallucinations[updateCounter].at(h).halId+".txt", all_files[i], false);
							hallucinations[updateCounter].at(h).setpredlabel(predlabel);
							hallucinations[updateCounter].at(h).writeTrajectoryFile();

						}
					}


				}
			}
			writeGTtrajectories(activityFrames,STEPSIZE,count);

			// compute the labels for the activity so far
			cout << "calling the classifier here" << endl;
			svm_classifier( 14 , &args[0] );
			// output prediction
			interpretPrediction("pred_"+all_files[i]+".txt",all_files[i],true);

			hallucinations.clear();
		}

	}
	// fclose(pRecFile);

	printf("ALL DONE.\n\n");

	return 0;
}

