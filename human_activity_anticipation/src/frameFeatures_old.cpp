//#include "frame.cpp"
#include "constants.h"
#include <iostream>
#include <fstream>


class FrameFeatures {
private:
    list<Frame> frames;

    vector<vector<double> > obj_features;
    vector<vector<double> > obj_obj_features;
    vector<double> skel_features;
    vector<vector<double> > skel_obj_features;

    vector<double> skel_temporal_features;
    vector<vector<double> > obj_temporal_features;

    FeaturesSkel* features_skeleton;

    int frameCount;


    //FeaturesSkelRGBD* features_rgbd;
    bool mirrored;
    bool temporal;
    bool temporalFlag;


    std::ofstream ofeatfile, sfeatfile, oofeatfile, sofeatfile, temporalSkelfeatfile, temporalObjfeatfile;

    void print_feats(vector<vector<double> > &feats) {
        for (size_t i = 0; i < feats.size(); i++) {
            cout << "object " << i << ":";
            for (size_t j = 0; j < feats.at(i).size(); j++) {
                cout << " " << feats.at(i).at(j);
            }
            cout << endl;
        }
    }

    void print_feats(vector<double> &feats, std::ofstream &file) {
        //cout << "feats: ";
        //file << "feats: ";
        for (size_t i = 0; i < feats.size(); i++) {
            //cout << " " << feats.at(i);
            file << "," << feats.at(i);
        }
        //cout << endl;
        file << endl;

    }

    void computeSkelFeatures(bool normalize) {

        bool started = false;
        int numFeats = 0;
        Frame frameNew = frames.back();
        started = features_skeleton->extractSkeletonFeature(frameNew.skeleton.data, frameNew.skeleton.pos_data);
        if (started) {
            skel_features = features_skeleton->getFeatureValues();
            //cout << "num of skel feats" << skel_features.size();
        }

    }

    void computeObjFeatures(bool normalize) {
        Frame frameNew = frames.back();
        for (size_t i = 0; i < frameNew.objects.size(); i++) {
            obj_features.push_back(vector<double>(0));
        }
       
       
        for (size_t i = 0; i < frameNew.objects.size(); i++) {
            obj_features.at(i) = frameNew.objects.at(i).features;
            obj_features.at(i).push_back(frameNew.objects.at(i).getCentroid().x);
            obj_features.at(i).push_back(frameNew.objects.at(i).getCentroid().y);
            obj_features.at(i).push_back(frameNew.objects.at(i).getCentroid().z);
        }
    }

    vector<double> computeObjObjFeatures( ObjectProfile & obj1,  ObjectProfile & obj2) {

        vector <double> features;
        // 
        features.push_back(obj1.centroid.x - obj2.centroid.x);
        features.push_back(obj1.centroid.y - obj2.centroid.y);
        features.push_back(obj1.centroid.z - obj2.centroid.z);
        features.push_back(obj1.getDistanceSqrBwCentroids(obj2));
        return features;
    }

    void computeObjPairFeatures(bool normalize) {
        // for every frame compute the obj-obj pair features
        Frame frameNew = frames.back();
        
        for (size_t i = 0; i < frameNew.objects.size(); i++) {
            for (size_t j = 0; j < frameNew.objects.size(); j++) {
                if (i != j) {
                    obj_obj_features.push_back(vector<double> (0));
                    obj_obj_features.at(obj_obj_features.size() - 1) = computeObjObjFeatures(frameNew.objects.at(i), frameNew.objects.at(j));

                }
            }
        }
    }
    
    vector<double> computeSkelObjFeatures(const Frame_skel & skel, const ObjectProfile & obj) {
        vector<double> features;

        
        double dist = 0;
        for (size_t i = 0; i < skel.transformed_joints.size(); i++) {
            dist = pow((skel.transformed_joints.at(i).x - obj.centroid.x), 2);
            dist += pow((skel.transformed_joints.at(i).y - obj.centroid.y), 2);
            dist += pow((skel.transformed_joints.at(i).z - obj.centroid.z), 2);
            features.push_back(dist);
        }
        return features;
    }

    void computeSkelObjPairFeatures(bool normalize) {
        // for every pair of objects compute the objObj features
        Frame frameNew = frames.back();
       
        for (size_t i = 0; i < frameNew.objects.size(); i++) {
            skel_obj_features.push_back(vector<double> (0));
            skel_obj_features.at(skel_obj_features.size() - 1) = computeSkelObjFeatures(frameNew.skeleton, frameNew.objects.at(i));
        }
    }

    void computeObjTemporalFeatures(bool normalize) {

        Frame frameNew = frames.back();
        Frame frameOld = frames.front();
        for (size_t i = 0; i < frameNew.objects.size(); i++) {
            obj_temporal_features.push_back(vector<double>(0));

            //frame->objects.at(i).printFeatures();
            obj_temporal_features.at(i).push_back(frameNew.objects.at(i).getXDispCentroids(frameOld.objects.at(i)));
            obj_temporal_features.at(i).push_back(frameNew.objects.at(i).getYDispCentroids(frameOld.objects.at(i)));
            obj_temporal_features.at(i).push_back(frameNew.objects.at(i).getVertDispCentroids(frameOld.objects.at(i)));
            obj_temporal_features.at(i).push_back(frameNew.objects.at(i).getDistanceSqrBwCentroids(frameOld.objects.at(i)));


            //TODO : add transformation as a feature 

            temporalObjfeatfile << frameNew.sequenceId << "," << frameOld.frameNum << "," << frameNew.frameNum << "," << frameNew.objects.at(i).objID;
            print_feats(obj_temporal_features.at(i), temporalObjfeatfile);
            //cout << "centroid x:" << objects.at(i).getCentroid().x << " y:" << objects.at(i).getCentroid().y << " z:" << objects.at(i).getCentroid().z << endl;
        }

    }

   /* void computeSkelTemporalFeatures(bool normalize) {
        Frame frameNew = frames.back();
        Frame frameOld = frames.front();
        // distance between of local joints positions
        for (size_t i = 0; i < frameNew.skeleton->num_local_joints; i++) {
            skel_temporal_features.push_back(frameNew.skeleton->joints_local[i][pos_data_x] - frameOld.skeleton->joints_local[i][pos_data_x]);
            skel_temporal_features.push_back(frameNew.skeleton->joints_local[i][pos_data_y] - frameOld.skeleton->joints_local[i][pos_data_y]);
            skel_temporal_features.push_back(frameNew.skeleton->joints_local[i][pos_data_z] - frameOld.skeleton->joints_local[i][pos_data_z]);
            double dist = pow((frameNew.skeleton->joints_local[i][pos_data_x] - frameOld.skeleton->joints_local[i][pos_data_x]), 2) + pow((frameNew.skeleton->joints_local[i][pos_data_y] - frameOld.skeleton->joints_local[i][pos_data_y]), 2) + pow((frameNew.skeleton->joints_local[i][pos_data_z] - frameOld.skeleton->joints_local[i][pos_data_z]), 2);
            skel_temporal_features.push_back(dist);

        }
        temporalSkelfeatfile << frameNew.sequenceId << "," << frameOld.frameNum << "," << frameNew.frameNum;
        print_feats(skel_temporal_features, temporalSkelfeatfile);
    }*/

    void computeSkelTemporalFeatures(bool normalize) {
        Frame frameNew = frames.back();
        Frame frameOld = frames.front();
        // distance between of local joints positions
        for (size_t i = 0; i < frameNew.skeleton.transformed_joints.size(); i++) {
            skel_temporal_features.push_back(frameNew.skeleton.transformed_joints.at(i).x - frameOld.skeleton.transformed_joints.at(i).x );
            skel_temporal_features.push_back(frameNew.skeleton.transformed_joints.at(i).y - frameOld.skeleton.transformed_joints.at(i).y);
            skel_temporal_features.push_back(frameNew.skeleton.transformed_joints.at(i).z - frameOld.skeleton.transformed_joints.at(i).z);
            double dist = pow((frameNew.skeleton.transformed_joints.at(i).x - frameOld.skeleton.transformed_joints.at(i).x), 2) + pow((frameNew.skeleton.transformed_joints.at(i).y - frameOld.skeleton.transformed_joints.at(i).y), 2) + pow((frameNew.skeleton.transformed_joints.at(i).z - frameOld.skeleton.transformed_joints.at(i).z), 2);
            skel_temporal_features.push_back(dist);

        }
        //CHECK : removing local joints
        /*for (size_t i = 0; i < frameNew.skeleton.num_local_joints; i++) {
            skel_temporal_features.push_back(frameNew.skeleton.joints_local[i][pos_data_x] - frameOld.skeleton.joints_local[i][pos_data_x]);
            skel_temporal_features.push_back(frameNew.skeleton.joints_local[i][pos_data_y] - frameOld.skeleton.joints_local[i][pos_data_y]);
            skel_temporal_features.push_back(frameNew.skeleton.joints_local[i][pos_data_z] - frameOld.skeleton.joints_local[i][pos_data_z]);
            double dist = pow((frameNew.skeleton.joints_local[i][pos_data_x] - frameOld.skeleton.joints_local[i][pos_data_x]), 2) + pow((frameNew.skeleton.joints_local[i][pos_data_y] - frameOld.skeleton.joints_local[i][pos_data_y]), 2) + pow((frameNew.skeleton.joints_local[i][pos_data_z] - frameOld.skeleton.joints_local[i][pos_data_z]), 2);
            skel_temporal_features.push_back(dist);

        }*/
        temporalSkelfeatfile << frameNew.sequenceId << "," << frameOld.frameNum << "," << frameNew.frameNum;
        print_feats(skel_temporal_features, temporalSkelfeatfile);
    }
    
    void writeObjObjFeats() {


        Frame frameNew = frames.back();
        int numObjs = frameNew.objects.size();
        int objPairCount = 0;
        for (size_t i = 0; i < numObjs; i++) {
            for (size_t j = 0; j < numObjs; j++) {
                if (i != j) {
                    oofeatfile << frameNew.sequenceId << "," << frameNew.frameNum << "," << frameNew.objects.at(i).objID << "," << frameNew.objects.at(j).objID;
                    print_feats(obj_obj_features.at(objPairCount), oofeatfile);
                    objPairCount++;
                }
            }

        }
    }

    void writeSkelObjFeats() {
        Frame frameNew = frames.back();
        int numObjs = frameNew.objects.size();

        for (size_t i = 0; i < numObjs; i++) {
            sofeatfile << frameNew.sequenceId << "," << frameNew.frameNum << "," << frameNew.objects.at(i).objID;
            print_feats(skel_obj_features.at(i), sofeatfile);
        }
    }

    void writeSkelFeats() {
        Frame frameNew = frames.back();
        sfeatfile << frameNew.sequenceId << "," << frameNew.frameNum;
        print_feats(skel_features, sfeatfile);
    }

    void writeObjFeats() {
        Frame frameNew = frames.back();
        int numObjs = frameNew.objects.size();

        for (size_t i = 0; i < numObjs; i++) {
            ofeatfile << frameNew.sequenceId << "," << frameNew.frameNum << "," << frameNew.objects.at(i).objID;
            print_feats(obj_features.at(i), ofeatfile);
        }
    }

public:
    void computeFreatures(bool normalize) {
        //if(frameCount <=1) {return;}
        obj_features.clear();
        obj_obj_features.clear();
        skel_features.clear();
        skel_obj_features.clear();
        features_skeleton->reset(false);
        
        computeSkelObjPairFeatures(normalize);
        computeObjFeatures(normalize);
        computeSkelFeatures(normalize);
        computeObjPairFeatures(normalize);

        if (temporal && temporalFlag && frameCount >1) {
            computeTemporalFreatures(normalize);
           // temporalFlag = false;
        }
        // write features
        writeObjFeats();
        writeSkelFeats();
        writeObjObjFeats();
        writeSkelObjFeats();
    }

    void computeTemporalFreatures(bool normalize) {

        obj_temporal_features.clear();
        skel_temporal_features.clear();
        computeObjTemporalFeatures(normalize);
        computeSkelTemporalFeatures(normalize);

    }

    void resetActivity() {
        frameCount = 0;
        frames.clear();

    }

    void setCurrentFrames(list<Frame> &f, int id) {
        frameCount++;
        frames = f;
       
        
    }
    void setCurrentFrame(Frame &f) {
        frameCount++;
        frames.push_back(f);
        if(frames.size()>2){frames.pop_front();}


    }

    FrameFeatures(bool Temporal,string name) {
        mirrored = false;
        temporal = Temporal;
        frameCount = 0;
        temporalFlag = true;//false;
        string filename = name + "_data_obj_feats.txt";
        ofeatfile.open(filename.c_str(), ios::app);
        filename = name + "_data_skel_feats.txt";
        sfeatfile.open(filename.c_str(), ios::app);
        filename = name+ "_data_obj_obj_feats.txt";
        oofeatfile.open(filename.c_str(), ios::app);
        filename = name + "_data_skel_obj_feats.txt";
        sofeatfile.open(filename.c_str(), ios::app);
        if (temporal) {
        	filename = name + "_data_temporal_obj_feats.txt";
            temporalObjfeatfile.open(filename.c_str(), ios::app);
            filename = name + "_data_temporal_skel_feats.txt";
            temporalSkelfeatfile.open(filename.c_str(), ios::app);
        }
        //  if (useSkeleton)
        features_skeleton = new FeaturesSkel(mirrored);

        //features_rgbd = new FeaturesSkelRGBD(mirrored);

    }

    ~FrameFeatures() {
        ofeatfile.close();
        sfeatfile.close();
        oofeatfile.close();
        sofeatfile.close();
        if (temporal) {
            temporalSkelfeatfile.close();
            temporalObjfeatfile.close();
        }

    }

};



