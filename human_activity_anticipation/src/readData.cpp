#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <map>
#include <vector>
using namespace std;

class readData {
private:

    int currentFrameNum;
    int currentFrameNum_RGBD;
    int currentFrameNum_Obj;
    int currentFrameNum_ObjPC;
    int lastFrame;
    bool skipOdd;
    string dataLocation;
    string dataLocation_mirrored;
    string fileName;
    string fileName_skeleton;
    string fileName_RGBD;
    string curActivity;
    vector<string> objectFeatureFileList;
    vector<string> objectPCFileList;
    map<string, string> data_act_map;
    ifstream* file;
    ifstream* file_RGBD;
    vector<ifstream*> file_objFeat;
    vector<ifstream*> file_objPC;
    bool mirrored;


    // print error message

    void errorMsg(string message, bool exitProgram) {
        cout << "ERROR! " << message << endl;
        printf("\tcurrentFrameNum = %d\n", currentFrameNum);
        printf("\tcurrentFrameNum_RGBD = %d\n", currentFrameNum_RGBD);

        if (exitProgram) {
            exit(1);
        }
    }

    void errorMsg(string message) {
        errorMsg(message, true);
    }

    bool parseChk(bool chk, bool skeleton) {
        if (!chk) {
            if (skeleton) {
                errorMsg("parsing error. (skeleton)", true);
            } else {
                errorMsg("parsing error. (RGBD) - IGNORE THIS ERROR!! (all random dataset will hit this error)", false);
            }
            return false;
        }
        return true;
    }

    // read skeleton data file

    void prepareSkeletonData() {
        //curActivity = data_act_map[fileName];(char*) fileName_skeleton.c_str()
        if (!mirrored) {
            fileName_skeleton = dataLocation + fileName + ".txt";
        } else {
            fileName_skeleton = dataLocation_mirrored + fileName + ".txt";
        }

        //printf("\tOpening \"%s\" (%s)\n", (char*) fileName_skeleton.c_str(), (char*) curActivity.c_str());
        printf("Trying to open %s\n", (char*) fileName_skeleton.c_str());
        file = new ifstream((char*) fileName_skeleton.c_str(), ifstream::in);
        currentFrameNum = -99;
    }

    void closeSkeletonData() {
        file->close();
        printf("\tskeleton file closed\n");
    }

    // return true if data retrieving was successful

    bool skipNextLine_skeleton() {
        string line;
        bool file_ended = true;
        if (getline(*file, line)) {
            file_ended = false;
            stringstream lineStream(line);
            string element;

            parseChk(getline(lineStream, element, ','), true);
            currentFrameNum = atoi((char*) element.c_str());
            //cout << "skipping frame " << currentFrameNum << " from skeleton data" << endl;
            if (element.compare("END") == 0) {
                file_ended = true;
                return false;
            }
        }


        if (currentFrameNum == -99) {
            errorMsg("file does not exist or empty!!");
        }

        return !file_ended;
    }

    bool readNextLine_skeleton(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF) {
        string line;
        bool file_ended = true;
        if (skipOdd) {
            if (getline(*file, line)) {
                file_ended = false;
                stringstream lineStream(line);
                string element;

                parseChk(getline(lineStream, element, ','), true);
                currentFrameNum = atoi((char*) element.c_str());
                //cout << "skipping frame " << currentFrameNum << " from skeleton data" << endl;
                if (element.compare("END") == 0) {
                    file_ended = true;
                    return false;
                }
            }
        }

        if (getline(*file, line)) {
            file_ended = false;
            stringstream lineStream(line);
            string element;

            int jointCount = 0;
            int joint_dataCount = 0;

            int pos_jointCount = 0;
            int pos_joint_dataCount = 0;

            parseChk(getline(lineStream, element, ','), true);
            currentFrameNum = atoi((char*) element.c_str());

            if (element.compare("END") == 0) {
                file_ended = true;
                return false;
            }

            while (getline(lineStream, element, ',')) {
                double e = strtod((char*) element.c_str(), NULL);

                if (jointCount < JOINT_NUM) {
                    data[jointCount][joint_dataCount] = e;
                    joint_dataCount++;

                    if (joint_dataCount == JOINT_DATA_ORI_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // ori conf value
                        data_CONF[jointCount][0] = atoi((char*) element.c_str());
                    } else if (joint_dataCount >= JOINT_DATA_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // pos conf value
                        data_CONF[jointCount][1] = atoi((char*) element.c_str());
                        jointCount++;
                        joint_dataCount = 0;
                    }

                } else {
                    // pos only joints
                    if (pos_jointCount >= POS_JOINT_NUM) {
                        errorMsg("PARSING ERROR!!!!!");
                    }
                    pos_data[pos_jointCount][pos_joint_dataCount] = e;
                    pos_joint_dataCount++;
                    if (pos_joint_dataCount >= POS_JOINT_DATA_NUM) {
                        parseChk(getline(lineStream, element, ','), true); // pos conf value
                        data_pos_CONF[pos_jointCount] = atoi((char*) element.c_str());

                        pos_jointCount++;
                        pos_joint_dataCount = 0;
                    }
                }
            }

            // check if there is more data in current frame..
            if (getline(lineStream, element, ',')) {
                errorMsg("more data exist in skeleton data ..\n");
            }

        }

        if (currentFrameNum == -99) {
            errorMsg("file does not exist or empty!!");
        }

        return !file_ended;
    }

    void prepareObjectData() {
        file_objFeat.resize(objectFeatureFileList.size());
        for (size_t i = 0; i < objectFeatureFileList.size(); i++) {
            cout << "\tOpening Object feature file " << i << endl;
            file_objFeat.at(i) = new ifstream((char*) objectFeatureFileList.at(i).c_str(), ifstream::in);
        }
        if (objectPCFileList.size() > 0) {
            file_objPC.resize(objectPCFileList.size());
            for (size_t i = 0; i < objectPCFileList.size(); i++) {
                cout << "\tOpening Object pc file " << i << ": " << objectPCFileList.at(i).c_str() << endl;
                file_objPC.at(i) = new ifstream((char*) objectPCFileList.at(i).c_str(), ifstream::in);
            }
        }
        currentFrameNum_Obj = -99;
    }

    void closeObjectData() {
        for (size_t i = 0; i < file_objFeat.size(); i++) {
            file_objFeat.at(i)->close();
            cout << "\tObject file " << i << " closed" << endl;
        }
        if (objectPCFileList.size() > 0) {
            for (size_t i = 0; i < objectPCFileList.size(); i++) {
                cout << "\tObject PC file " << i << " closed" << endl;
                file_objPC.at(i)->close();
            }
        }
    }

    bool skipNextLine_ObjectData() {
    	string line;
    	bool file_ended = true;
        for (size_t i = 0; i < file_objFeat.size(); i++) {

        if (getline(*file_objFeat.at(i), line)) {
            file_ended = false;



        }



        }
        return !file_ended;

    }

    bool readNextLine_ObjectData(vector < vector<double> > &objFeats) {
        objFeats.clear();
        objFeats.resize(file_objFeat.size());
        for (size_t i = 0; i < file_objFeat.size(); i++) {
            string line;
            char* line_c;
            bool file_ended = true;

            if (getline(*file_objFeat.at(i), line)) {
                file_ended = false;

                line_c = (char*) line.c_str();
                char* element = strtok(line_c, ",");
                if (element == NULL || strcmp(element, "END") == 0) {
                    file_ended = true;
                    return false;
                }
                currentFrameNum_Obj = atoi(element);
                objFeats.at(i).push_back(double(currentFrameNum_Obj));
                if (currentFrameNum_Obj != currentFrameNum) {
                    printf("Object: %d skeleton: %d\n", currentFrameNum_Obj, currentFrameNum);
                    errorMsg("FRAME NUMBER BETWEEN OBJECT AND SKELETON DOES NOT MATCH!!!!!!!!! (READING OBJECT FILE)");
                }

                for (int y = 0; y < NUM_OBJ_FEATS - 1; y++) {
                    element = strtok(NULL, ","); // passing NULL keeps tokenizing previous call
                    if (element == NULL) {
                        file_ended = true;
                        return false;
                    }
                    double e = atof(element);
                    objFeats.at(i).push_back(e);

                }
                // check if there is more data in current frame..

                element = strtok(NULL, ",");
                if (element != NULL) {
                    printf("line_c = %s\n", line_c);
                    errorMsg("more data exist in image data ..\n");

                }
            }
        }
        return true;
    }
    
    bool skipNextLine_ObjectPCData() {
        	string line;
        	bool file_ended = true;
        	for (size_t i = 0; i < file_objPC.size(); i++) {

        		if (getline(*file_objPC.at(i), line)) {
                file_ended = false;

            }

            }
            return !file_ended;

        }

    bool readNextLine_ObjectPCData(vector < vector<int> > &objPCIndices) {
        objPCIndices.clear();
        objPCIndices.resize(file_objPC.size());
        for (size_t i = 0; i < file_objPC.size(); i++) {
            string line;
            char* line_c;
            bool file_ended = true;

            if (getline(*file_objPC.at(i), line)) {
                file_ended = false;

                line_c = (char*) line.c_str();
                char* element = strtok(line_c, ",");
                if (element == NULL || strcmp(element, "END") == 0) {
                    file_ended = true;
                    return false;
                }
                
                element = strtok(NULL, ",");
                currentFrameNum_ObjPC = atoi(element);
                
                if (currentFrameNum_ObjPC != currentFrameNum) {
                    printf("Object: %d skeleton: %d\n", currentFrameNum_ObjPC, currentFrameNum);
                    errorMsg("FRAME NUMBER BETWEEN OBJECT PC AND SKELETON DOES NOT MATCH!!!!!!!!! (READING OBJECT PC FILE)");
                }
                element = strtok(NULL, ",");
                int objId = atoi(element);
                
                while(element != NULL) {
                    element = strtok(NULL, ","); // passing NULL keeps tokenizing previous call
                    if(element != NULL) {
                      int e = atoi(element);
                      objPCIndices.at(i).push_back(e);
                    }
                }
                
            }
        }
        return true;
    }

    // read RGBD data file

    void prepareRGBDData() {
        fileName_RGBD = dataLocation + fileName + "_rgbd.txt";
        //printf("\tOpening \"%s\" (%s)\n", (char*) fileName_RGBD.c_str(), (char*) curActivity.c_str());
        file_RGBD = new ifstream((char*) fileName_RGBD.c_str(), ifstream::in);
        currentFrameNum = -99;
    }

    void closeRGBDData() {
        file_RGBD->close();
        printf("\tRGBD file closed\n");
    }

    // return true if data retrieving was successful

    bool readNextLine_RGBD(int ***IMAGE) {
        string line;
        char* line_c;
        bool file_ended = true;

        if (skipOdd) {
            if (getline(*file_RGBD, line)) {
                file_ended = false;

                line_c = (char*) line.c_str();
                char* element = strtok(line_c, ",");
                if (element == NULL || strcmp(element, "END") == 0) {
                    file_ended = true;
                    return false;
                }
            }
        }

        if (getline(*file_RGBD, line)) {
            file_ended = false;

            line_c = (char*) line.c_str();
            char* element = strtok(line_c, ",");
            if (element == NULL || strcmp(element, "END") == 0) {
                file_ended = true;
                return false;
            }
            currentFrameNum_RGBD = atoi(element);
            if (currentFrameNum != currentFrameNum_RGBD) {
                printf("skeleton: %d rgbd: %d\n", currentFrameNum, currentFrameNum_RGBD);
                errorMsg("FRAME NUMBER BETWEEN SKELETON AND RGBD DOES NOT MATCH!!!!!!!!! (READING RGBD)");
            }
            for (int y = 0; y < Y_RES; y++) {
                for (int x = 0; x < X_RES; x++) {
                    for (int d = 0; d < RGBD_data; d++) {

                        element = strtok(NULL, ","); // passing NULL keeps tokenizing previous call
                        if (element == NULL) {
                            file_ended = true;
                            return false;
                        }
                        int e = atoi(element);

                        if (!mirrored) {
                            IMAGE[x][y][d] = e;
                        } else {
                            IMAGE[x][(Y_RES - 1) - y][d] = e;
                        }
                    }
                    
                }
            }
            //printf( "x: %d y: %d" ,x,y);
            // check if there is more data in current frame..
            element = strtok(NULL, ",");
            if (element != NULL) {
                printf("line_c = %s\n", line_c);
                errorMsg("more data exist in RGBD data ..\n");

            }

        }

        return !file_ended;
    }

    bool skipNextLine_RGBD() {
        string line;
        char* line_c;
        bool file_ended = true;


        if (getline(*file_RGBD, line)) {
            file_ended = false;

            line_c = (char*) line.c_str();
            char* element = strtok(line_c, ",");
            if (element == NULL || strcmp(element, "END") == 0) {
                file_ended = true;
                return false;
            }
            currentFrameNum_RGBD = atoi(element);
        }


        return !file_ended;
    }

public:



    // return true if data retrieving was successful

    int readNextFrame(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF, int ***IMAGE, vector<vector<double> > &objFeats) {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = readNextLine_skeleton(data, pos_data, data_CONF, data_pos_CONF);
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }
        bool status_obj = readNextLine_ObjectData(objFeats);
        bool status_RGBD = readNextLine_RGBD(IMAGE);
        if (status_RGBD) {
            lastFrame = currentFrameNum;
        } else {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
        }
        if (status_RGBD)
            return currentFrameNum;
        else
            return 0;
    }
    
        int readNextFrame(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF, int ***IMAGE, vector<vector<double> > &objFeats, vector<vector<int> > &objPCInds) {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = readNextLine_skeleton(data, pos_data, data_CONF, data_pos_CONF);
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }
        bool status_obj = readNextLine_ObjectData(objFeats);
        bool status_objPC = readNextLine_ObjectPCData(objPCInds); 
        bool status_RGBD = readNextLine_RGBD(IMAGE);
        if (status_RGBD) {
            lastFrame = currentFrameNum;
        } else {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
        }
        if (status_RGBD)
            return currentFrameNum;
        else
            return 0;
    }

    readData(string dataLoc, string fileN, map<string, string> d_a_map, int i, bool mirrored, string dataLoc_mirrored, bool skip, vector<string> objectFeatureFiles) {
        if (!mirrored) {
            printf("%d. ", i);
        } else {
            printf("%d(M). ", i);
        }
        dataLocation = dataLoc;
        dataLocation_mirrored = dataLoc_mirrored;
        fileName = fileN;
        data_act_map = d_a_map;
        this->mirrored = mirrored;
        skipOdd = skip;
        this->objectFeatureFileList = objectFeatureFiles;
        prepareSkeletonData();
        prepareRGBDData();
        prepareObjectData();

    }
    readData(string dataLoc, string fileN, map<string, string> d_a_map, int i, bool mirrored, string dataLoc_mirrored, bool skip, vector<string> objectFeatureFiles, vector<string> objectPCFiles) {
        if (!mirrored) {
            printf("%d. ", i);
        } else {
            printf("%d(M). ", i);
        }
        dataLocation = dataLoc;
        dataLocation_mirrored = dataLoc_mirrored;
        fileName = fileN;
        data_act_map = d_a_map;
        this->mirrored = mirrored;
        skipOdd = skip;
        this->objectFeatureFileList = objectFeatureFiles;
        this->objectPCFileList = objectPCFiles;
        prepareSkeletonData();
        prepareRGBDData();
        prepareObjectData();

    }

    readData(string dataLoc, string fileN, bool skip, vector<string> objectFeatureFiles) {
        dataLocation = dataLoc;
        fileName = fileN;
        this->mirrored = false; //this was previously unitialized

        skipOdd = skip;
        this->objectFeatureFileList = objectFeatureFiles;
        prepareSkeletonData();
        prepareRGBDData();
        prepareObjectData();

    }

    readData(string dataLoc, string fileN) {
        dataLocation = dataLoc;
        fileName = fileN;
        skipOdd = false;
        this->mirrored = false; //this was previously unitialized
        prepareSkeletonData();
        prepareRGBDData();
    }

    int readNextFrame(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF, int ***IMAGE) {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = readNextLine_skeleton(data, pos_data, data_CONF, data_pos_CONF);
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }

        bool status_RGBD = readNextLine_RGBD(IMAGE);
        if (status_RGBD) {
            lastFrame = currentFrameNum;
        } else {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
        }
        if (status_RGBD)
            return currentFrameNum;
        else
            return 0;
    }

        int readNextFrame(double **data, double **pos_data, int **data_CONF, int *data_pos_CONF) {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = readNextLine_skeleton(data, pos_data, data_CONF, data_pos_CONF);
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }

      
        if (status)
            return currentFrameNum;
        else
            return 0;
    }
    
    int skipNextFrame() {
        if (currentFrameNum % 100 == 0) {
            printf("\t\t(progress..) frame num = %d\n", currentFrameNum);
        }
        bool status = skipNextLine_skeleton();
        if (!status) {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
            return false;
        }

        bool status_RGBD = skipNextLine_RGBD();
        if (status_RGBD) {
            lastFrame = currentFrameNum;
        } else {
            printf("\t\ttotal number of frames = %d\n", lastFrame);
        }
        bool status_obj = skipNextLine_ObjectData();
        bool status_objPC = skipNextLine_ObjectPCData();

        if (status_RGBD)
            return currentFrameNum;
        else
            return 0;
    }

    readData() {

    }

    ~readData() {
        closeSkeletonData();
        closeRGBDData();
        closeObjectData();
        printf("\n");
    }

};


// EXTREMLEY SLOW RGBD PARSING CODE..
// NOW REPLACED WITH MUCH FASTER CODE!!


// return true if data retrieving was successful
/*
bool readNextLine_RGBD(int ***IMAGE) {
string line;        
bool file_ended = true;

if (getline(*file_RGBD,line)) {
    file_ended = false;
    stringstream lineStream(line);
    string element;
    
    bool status = parseChk(getline(lineStream, element, ','), false);
    if (!status) {
        file_ended = true;
        return false;                
    }
    if (element.compare("END") == 0) {
        file_ended = true;
        return false;
    }
    currentFrameNum_RGBD = atoi((char*)element.c_str());
    if (currentFrameNum != currentFrameNum_RGBD) {
        printf("skeleton: %d rgbd: %d\n", currentFrameNum, currentFrameNum_RGBD);
        errorMsg("FRAME NUMBER BETWEEN SKELETON AND RGBD DOES NOT MATCH!!!!!!!!! (READING RGBD)");
    }

    for (int y=0;y<Y_RES;y++) {
        for (int x=0;x<X_RES;x++) {   
            for (int d = 0; d<RGBD_data; d++) {         
                status = parseChk(getline(lineStream, element, ','), false);
                if (!status) {
                    file_ended = true;
                    return false;
                }
                int e = atoi((char*)element.c_str());
                
                if (!mirrored) {
                    IMAGE[x][y][d] = e;
                } else {
                    IMAGE[x][(Y_RES-1)-y][d] = e; 
                }
            }
        }
    }
    
    // check if there is more data in current frame..
    if (getline(lineStream, element,',')) {
        errorMsg("more data exist in RGBD data ..\n");
    }
    
} 

return !file_ended;
}
 */
