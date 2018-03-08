#ifndef CONSTANTS_H__
#define CONSTANTS_H__
const int SLEEP_TIME = 0;

const int JOINT_NUM = 11;
const int JOINT_DATA_ORI_NUM = 9;
const int JOINT_DATA_POS_NUM = 3;
const int JOINT_DATA_NUM = (JOINT_DATA_ORI_NUM+JOINT_DATA_POS_NUM);
const int JOINT_DATA_TYPE_NUM = 2; // two types : orientation and xyz position


const int HEAD_JOINT_NUM = 0;
const int NECK_JOINT_NUM = 1;
const int TORSO_JOINT_NUM = 2;
const int LEFT_SHOULDER_JOINT_NUM = 3;
const int LEFT_ELBOW_JOINT_NUM = 4;
const int RIGHT_SHOULDER_JOINT_NUM = 5;
const int RIGHT_ELBOW_JOINT_NUM =6;
const int LEFT_HIP_JOINT_NUM = 7;
const int LEFT_KNEE_JOINT_NUM = 8;
const int RIGHT_HIP_JOINT_NUM = 9;
const int RIGHT_KNEE_JOINT_NUM = 10;


const int POS_JOINT_NUM = 4;
const int POS_JOINT_DATA_NUM = 3;

const int POS_LEFT_HAND_NUM = 0;
const int POS_RIGHT_HAND_NUM = 1;
const int POS_LEFT_FOOT_NUM = 2;
const int POS_RIGHT_FOOT_NUM = 3;

const int X_RES = 640;//320;
const int Y_RES = 480;//240;
const int RGBD_data = 4;

const int NUM_OBJ_FEATS = 12;
//const int NUM_OBJ_FEATS = 18;


// 30 fps
//const int frameStoreNum = 66;
//const int compareFrame[] = {0, -5, -9, -14, -20, -27, -35, -44, -54, -65};

const int frameStoreNum = 1;// 21;
const int compareFrame[] = {};//{0, -4, -8, -12, -16, -20};

//const int frameStoreNum = 5;// 21;
//const int compareFrame[] = {0,-1,-2,-3,-4};//{0, -4, -8, -12, -16, -20};


//const int frameStoreNum = 91;
//const int compareFrame[] = {0, -5, -9, -14, -20, -27, -35, -44, -54, -65, -77, -90};
const int compareFrameNum = sizeof(compareFrame)/sizeof(compareFrame[0]);

#define pos_data_x 0
#define pos_data_y 2
#define pos_data_z 1


#define data_x 9
#define data_y 11
#define data_z 10

#endif
