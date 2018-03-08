#include <vector>
#ifndef FRAME_SKEL__H
#define FRAME_SKEL__H
#include <pcl/point_types.h>

#include "includes/point_types.h"
typedef pcl::PointXYZRGB PointT;

#include "includes/CombineUtils.h"

class Frame_skel{
public:
    //double data[JOINT_NUM][JOINT_DATA_NUM];
   // double data_conf[];
    //double pos_data[POS_JOINT_NUM][POS_JOINT_DATA_NUM];
   // double data_pos_conf[];
    double **data; //[JOINT_NUM][JOINT_DATA_NUM];
    int **data_CONF; //[JOINT_NUM][JOINT_DATA_TYPE_NUM]
    double **pos_data; //[POS_JOINT_NUM][POS_JOINT_DATA_NUM];
    int *pos_data_CONF; //[POS_JOINT_NUM]
    int num_local_joints; 
    vector<pcl::PointXYZ> transformed_joints;
    pcl::PointXYZ headOrientation;

    double ** joints_local;
    vector<int> jointList;
    vector<int> pos_jointList;
    int frameId;
    
     double* computeLocalLoc(double head_ori[9], double head_pos[3], double hand_pos[3]) {
        double handx = hand_pos[0] - head_pos[0];
        double handy = hand_pos[1] - head_pos[1];
        double handz = hand_pos[2] - head_pos[2];
        
        double* rel_hand = new double[3];
        
        rel_hand[0] = (head_ori[0]*handx + head_ori[3]*handy + head_ori[6]*handz) ; /// 1000;
        rel_hand[1] = (head_ori[1]*handx + head_ori[4]*handy + head_ori[7]*handz);//  / 1000;
        rel_hand[2] = (head_ori[2]*handx + head_ori[5]*handy + head_ori[8]*handz); // / 1000;
      
        
        //printf("%.2f \t%.2f \t%.2f\n", rel_hand[0], rel_hand[1], rel_hand[2]);
        
        return rel_hand;
    }
    
    void computePosition() {
        //double data[JOINT_NUM][JOINT_DATA_NUM], double pos_data[POS_JOINT_NUM][POS_JOINT_DATA_NUM], int variableNum) {
        
        // compute hand loc
        double left_hand_pos[3];
        double right_hand_pos[3];
        double head_ori[9];
        double head_pos[3];
        double * local_joint;
        double joint_pos[3];
        
        
        

        for (int i = 0;i<9;i++) {
            head_ori[i]=data[HEAD_JOINT_NUM][i];
        }
        for (int i=0;i<3;i++) {
            head_pos[i]=data[HEAD_JOINT_NUM][i+9];
        }
        
        for (size_t i = 0; i < jointList.size(); i ++){
            for (int j=0;j<3;j++) {
                joint_pos[j]=data[jointList.at(i)][j+9];
            }
            local_joint = computeLocalLoc(head_ori, head_pos, joint_pos);
            for (int j=0; j<3; j++){
                joints_local[i][j] = local_joint[j];
            }
        }
        for (int i=0;i<3;i++) {
            left_hand_pos[i]=pos_data[POS_LEFT_HAND_NUM][i];
        }
        for (int i=0;i<3;i++) {
            right_hand_pos[i]=pos_data[POS_RIGHT_HAND_NUM][i];
        }
        
        
        local_joint = computeLocalLoc(head_ori, head_pos, left_hand_pos);
        for (int j=0; j<3; j++){
            joints_local[jointList.size()][j] = local_joint[j];
        }
        local_joint = computeLocalLoc(head_ori, head_pos, right_hand_pos);
        for (int j=0; j<3; j++){
            joints_local[jointList.size()+1][j] = local_joint[j];
        }
        
        return;
    } // end computeHandPosition
    
    void transformJointPositions(string transformFile)
    {
        TransformG globalTransform;
        globalTransform = readTranform(transformFile);
        for (size_t i = 0; i < jointList.size(); i++) {
                //pcl::PointXYZ p1 (data[i][9],data[i][11],data[i][10]);
                pcl::PointXYZ pt ;
                pt.x = data[jointList.at(i)][9];
                pt.y = data[jointList.at(i)][11];
                pt.z = data[jointList.at(i)][10];
    
                globalTransform.transformPointInPlace(pt);
                transformed_joints.push_back(pt);
        }
        for (size_t i = 0; i < pos_jointList.size(); i++) {
                //pcl::PointXYZ p1 (data[i][9],data[i][11],data[i][10]);
                pcl::PointXYZ pt ;
                pt.x = pos_data[pos_jointList.at(i)][0];
                pt.y = pos_data[pos_jointList.at(i)][2];
                pt.z = pos_data[pos_jointList.at(i)][1];
    
                globalTransform.transformPointInPlace(pt);
                transformed_joints.push_back(pt);
        }
   
    }
    
    void initialize(double **data_, double **pos_data_, string transformFile){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }

         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
        }
        computePosition();
        transformJointPositions(transformFile);
    }

    void initialize_partial(double **data_, double **pos_data_, string transformFile){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        //jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        //jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }

         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
        }
        computePosition();
        transformJointPositions(transformFile);
    }

   void initialize(double **data_, double **pos_data_){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }

         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
		}
		computePosition();
		double head_ori[9];
		for (int i = 0; i < 9; i++) {
			head_ori[i] = data[HEAD_JOINT_NUM][i];
		}
		 pcl::PointXYZ ph;
		 ph.x =data[HEAD_JOINT_NUM][9];
		 ph.y =data[HEAD_JOINT_NUM][11];
		 ph.z =data[HEAD_JOINT_NUM][10];
		headOrientation.x = ph.x - 100 * head_ori[2];
		headOrientation.y = ph.y - 100 * head_ori[8];
		headOrientation.z = ph.z - 100 * head_ori[5];

	}

    Frame_skel(double **data_, double **pos_data_, string transformFile){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }
        
         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {        
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
        }
        computePosition();
        transformJointPositions(transformFile);
    }
    Frame_skel(double **data_, double **pos_data_){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }
        
         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {        
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
		}
		computePosition();
		double head_ori[9];
		for (int i = 0; i < 9; i++) {
			head_ori[i] = data[HEAD_JOINT_NUM][i];
		}
		 pcl::PointXYZ ph;
		 ph.x =data[HEAD_JOINT_NUM][9];
		 ph.y =data[HEAD_JOINT_NUM][11];
		 ph.z =data[HEAD_JOINT_NUM][10];
		headOrientation.x = ph.x - 100 * head_ori[2];
		headOrientation.y = ph.y - 100 * head_ori[8];
		headOrientation.z = ph.z - 100 * head_ori[5];

	}
    
    Frame_skel(double **data_, double **pos_data_, string transformFile, int fid){
        // initialize joint list
    	jointList.push_back(HEAD_JOINT_NUM);
        jointList.push_back(NECK_JOINT_NUM);
        jointList.push_back(TORSO_JOINT_NUM);
        jointList.push_back(LEFT_SHOULDER_JOINT_NUM);
        jointList.push_back(LEFT_ELBOW_JOINT_NUM);
        jointList.push_back(RIGHT_SHOULDER_JOINT_NUM);
        jointList.push_back(RIGHT_ELBOW_JOINT_NUM);
        pos_jointList.push_back(POS_LEFT_HAND_NUM);
        pos_jointList.push_back(POS_RIGHT_HAND_NUM);
        num_local_joints = jointList.size()+pos_jointList.size();
        // initialize data
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
        joints_local = new double*[num_local_joints];
        for (int i = 0; i < num_local_joints; i ++){
            joints_local[i] = new double[3];
        }
        
         // store current data
        for (int i=0;i<JOINT_NUM;i++) {
            for (int j=0;j<JOINT_DATA_NUM;j++) {
                data[i][j] = data_[i][j];
            }
        }

        for (int i=0;i<POS_JOINT_NUM;i++) {        
            for (int j=0;j<POS_JOINT_DATA_NUM;j++) {
                pos_data[i][j] = pos_data_[i][j];
            }
        }
        computePosition();
        transformJointPositions(transformFile);
        frameId = fid;
        TransformG globalTransform;
        globalTransform = readTranform(transformFile);
        double head_ori[9];
        for (int i = 0;i<9;i++) {
            head_ori[i]=data[HEAD_JOINT_NUM][i];
        }
        pcl::PointXYZ ph;
        ph.x =data[HEAD_JOINT_NUM][9];
        ph.y =data[HEAD_JOINT_NUM][11];
        ph.z =data[HEAD_JOINT_NUM][10];

        headOrientation.x = ph.x - 100*head_ori[2];
        headOrientation.y = ph.y - 100*head_ori[8];
        headOrientation.z = ph.z - 100*head_ori[5];
        globalTransform.transformPointInPlace(headOrientation);

    }
    Frame_skel(){

    }

    ~Frame_skel(){
    	//delete(data);
    	//delete(data_CONF);
    	//delete(pos_data);
    	//delete(pos_data_CONF);
    	//delete(joints_local);
    }
};
 
#endif

