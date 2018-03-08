
#include <vector>
#include <assert.h>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "frame_skel.h"
//#include "Point2D.h"
//#include "HOG.cpp"
//#include "HOGFeaturesOfBlock.h"
//#include "includes/point_types.h"

//#include <sensor_msgs/point_cloud_conversion.h>
//#include <>
#include "includes/color.cpp"

//#include <point_cloud_mapping/geometry/nearest.h>
#include <iostream>



typedef pcl::PointXYZRGB PointT;
#include "includes/point_types.h"
#include "includes/CombineUtils.h"

#define sqr(x) ((x)*(x))

using namespace std;

class ObjectProfile {
    vector<float> eigenValues; // sorted in ascending order
public:
    vector<double> features;
    pcl::PointCloud<PointT> cloud;
    //HOGFeaturesOfBlock avgHOGFeatsOfObject;
    float avgH;
    float avgS;
    float avgV;
    int minX, minY, maxX, maxY;
    int objID;
    vector<int> pcInds;
    string transformfile;
    string objectType;

    pcl::PointXYZ centroid;
    pcl::PointXYZ center;
    Eigen::Vector3d normal;

    ObjectProfile(vector<double> & feats, pcl::PointCloud<PointT> &fullCloud, map<int, int> &tablePoints, int id, string transFile) ;
    
    ObjectProfile(vector<double> & feats, pcl::PointCloud<PointT> &fullCloud, map<int, int> &tablePoints, int id);


    ObjectProfile(vector<double> & feats, pcl::PointCloud<PointT> &fullCloud, map<int, int> &tablePoints, int id, string transFile, vector<int> &PCInds) ;


    void initialize();

    void setObjectType(string);

    string getObjectType();


    void getObjectPointCloud(pcl::PointCloud<PointT> &fullCloud, vector<int> &PCInds);


    void getObjectPointCloud(pcl::PointCloud<PointT> &fullCloud, map<int, int> &tablePoints);


    void filterCloud(map<int, int> &tablePoints);

    void setEigValues(Eigen::Vector3d eigenValues_);

    float getDescendingLambda(int index) ;


    void computeCentroid();


    double getMinDistanceTo(pcl::PointXYZ p);

    
    double getDistanceToCentroid(pcl::PointXYZ p);

        
    pcl::PointXYZ getCentroid() ;
    void setCentroid(pcl::PointXYZ) ;

    void computeCenter();

    pcl::PointXYZ getCenter();


    float getScatter() ;


    float getLinearNess() ;


    float getPlanarNess();


    float getNormalZComponent() ;


    float getAngleWithVerticalInRadians();


    float getHorzDistanceBwCentroids( ObjectProfile & other) ;


    float getDistanceSqrBwCentroids( ObjectProfile & other);

    float getDistanceSqrBwCenters(const ObjectProfile & other);


    float getVertDispCentroids(const ObjectProfile & other);

    float getXDispCentroids(const ObjectProfile & other);

    float getYDispCentroids(const ObjectProfile & other) ;

    
    float getVertDispCenters(const ObjectProfile & other);


    float getHDiffAbs(const ObjectProfile & other) ;


    float getSDiff(const ObjectProfile & other);
    float getVDiff(const ObjectProfile & other);

    float getAngleDiffInRadians(const ObjectProfile & other) ;

    float getNormalDotProduct(const ObjectProfile & other) ;

    float getInnerness(const ObjectProfile & other);

    float pushHogDiffFeats(const ObjectProfile & other, vector<float> & feats);
    float getCoplanarity(const ObjectProfile & other) ;

    void printFeatures() ;
    void setFeatures(vector<double> &feat);




    /*  int getConvexity(const SpectralProfile & other, float mindistance) {
          VectorG centroid1(centroid.x, centroid.y, centroid.z);
          VectorG centroid2(other.centroid.x, other.centroid.y, other.centroid.z);

          VectorG c1c2 = centroid2.subtract(centroid1);
          VectorG c2c1 = centroid1.subtract(centroid2);
          VectorG normal1(normal[0], normal[1], normal[2]);
          VectorG normal2(other.normal[0], other.normal[1], other.normal[2]);
          if (mindistance < 0.04 && ((normal1.dotProduct(c1c2) <= 0 && normal2.dotProduct(c2c1) <= 0) || fabs(normal1.dotProduct(normal2)) > 0.95)) // refer local convexity criterion paper
          {
              return 1;
          }
          // else return 0
          return 0;
      }*/

};

class Frame {
private:


    vector<vector<double> > objFeats;
    //HOG hog;
    //std::vector<HOGFeaturesOfBlock> aggHogVec;
    static const int BLOCK_SIDE = 8;
    map<int, int> tablePoints;
    bool findTable;
    

    void createPointCloud(int ***IMAGE, string transformfile);

    
        void createPointCloud(int ***IMAGE) ;
       

    /* This function takes a HOG object and aggregates the HOG features for each stripe in the chunk.
     It populates aggHogVec with one HOGFeaturesOfBlock object for each stripe in the image. */
//    void computeAggHogBlock(int numStripes, int minXBlock, int maxXBlock, int minYBlock, int maxYBlock, HOGFeaturesOfBlock &hogObject);


 //   void computeObjectHog();


   // void computeHogDescriptors() ;




public:
    static int FrameNum;
    int frameNum;
    string sequenceId;
    vector<ObjectProfile> objects;
    Frame_skel skeleton;
    pcl::PointCloud<PointT> cloud;
    vector<double> rgbdskel_feats;


    //void printHOGFeats();


    void savePointCloud();


    void saveObjImage(ObjectProfile & obj, int ***IMAGE);


    void saveImage();


    // MIRRORED means skeleton is mirrored; RGBD comes in non mirrored form
    // but mirroring should be easy for RGBD

    Frame();
    
    Frame(int ***IMAGE, double** data, double **pos_data, vector<vector<double> > &objFeats, string seqId, int fnum, string transformfile);
    

     Frame(int ***IMAGE, double** data, double **pos_data, vector<vector<double> > &objFeats, string seqId, int fnum) ;
     
       
    Frame(int ***IMAGE, double** data, double **pos_data, vector<vector<double> > &objFeats, string seqId, int fnum, string transformfile, vector<vector<int> > &objPCInds) ;

    Frame(int ***IMAGE, double** data, double **pos_data, vector<vector<double> > &objFeats, string seqId, int fnum, string transformfile, vector<vector<int> > &objPCInds, vector<string> types);
    Frame(int ***IMAGE, double** data, double **pos_data, vector<vector<double> > &objFeats, string seqId, int fnum, string transformfile, vector<vector<int> > &objPCInds, bool partial) ;

    ~Frame();
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
