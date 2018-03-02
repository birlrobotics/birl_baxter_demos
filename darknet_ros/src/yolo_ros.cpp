#include "yolo_ros.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_ros/bbox_array.h>//调用bbox_array消息类型
#include <darknet_ros/bbox.h>//调用bbox消息类型

extern "C" {
  #include "box.h"
}

// initialize YOLO functions that are called in this script
//ROS_box *run_yolo();
PredBox *run_yolo();
void load_net(char *cfgfile, char *weightfile, float thresh, float hier);
int get_obj_count();

// 设置yolo的配置文件和权重，weights文件夹里只有tiny_yolo.weights和yolo.weights两种
char *cfg =(char*) "/home/will/ros_ws/src/darknet_ros/cfg/tiny-yolo-voc.cfg";//use tiny-yolo-voc.cfg will faster than yolo.cfg
//char *cfg = "/home/will/ros_ws/src/darknet_ros/cfg/tiny-yolo-voc.cfg";
char *weights =(char*) "/home/will/ros_ws/src/darknet_ros/weights/tiny-yolo-voc.weights";//use tiny-yolo-voc.weights will faster than yolo.weights
//char *weights = "/home/will/ros_ws/src/darknet_ros/weights/tiny-yolo-voc.weights";
float thresh = 0.5;

const std::string class_labels[] = { "person", "bicycle", "car", "chair", "aeroplane", "bus", "train", "potted plant",
		     	             "boat", "tv monitor", "dining table", "birl", "sofa", "motorbike", "bottle",
		                     "cat", "dog", "horse", "sheep", "cow" };//开头原来为"aeroplane"，但识别出的人成了"aeroplane"，是否每个标签的顺序是有含义的，不能随意更改？2.互换chair和boat，3.互换car和birl，4.互换cat和potted plant，5.cow和TV monitor。6.train和birl，7.horse和sofa，8.sheep和train，9.dog和sheep，10.aeroplane和bottle，11.birl和sheep，12.train和dog
const int num_classes = sizeof(class_labels)/sizeof(class_labels[0]);

cv::Mat input_image;

//设置参数，使用USBCAM或Kinect，（1,2,3）Kinect用，（①，②，③）USBCAM用
const std::string CAMERA_TOPIC_NAME = "/camera/rgb/image_raw";//1
const std::string CAMERA_WIDTH_PARAM ="/yolo_ros/image_width";//2
const std::string CAMERA_HEIGHT_PARAM ="/yolo_ros/image_height";//3，这三个在Kinect时用
//const std::string CAMERA_TOPIC_NAME = "/usb_cam/image_raw";//①
//const std::string CAMERA_WIDTH_PARAM = "/usb_cam/image_width";//②
//const std::string CAMERA_HEIGHT_PARAM = "/usb_cam/image_height";//③，这三个在USBCAM时用
const std::string OPENCV_WINDOW = "YOLO object detection";
int FRAME_W;
int FRAME_H;
int FRAME_AREA;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage

// Mat和iplimage两者对于内存图像数据创建稍有不同：IplImage，通过cvCreateImage，创建后复制像素到创建的内存，
//cv::Mat，直接可以通过构造函数Mat(int _rows, int _cols, int _type, void* _data, size_t _step=AUTO_STEP);直接创建。
IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(input_image);//将Mat格式的转换成IplImage格式
   return ROS_img;
}

class yoloObjectDetector
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;
   ros::Publisher _found_object_pub;
   ros::Publisher _bboxes_pub;
   //std::vector< std::vector<PredBox> > _class_bboxes;
   //std::vector<int> _class_obj_count;
   std::vector<cv::Scalar> _bbox_colors;
   darknet_ros::bbox_array _bbox_results_msg;//调用bbox_array消息类型，不要漏了头文件darknet_ros/bbox_array.h
   PredBox* _boxes;

public:
   yoloObjectDetector() : _it(_nh), _bbox_colors(num_classes)
   {
      int incr = floor(255/num_classes);
      for (int i = 0; i < num_classes; i++)
      {
         _bbox_colors[i] = cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i);
      }

      _image_sub = _it.subscribe(CAMERA_TOPIC_NAME, 1,
	                       &yoloObjectDetector::imageCallback,this);
      _found_object_pub = _nh.advertise<std_msgs::Int8>("object_detector", 1);//发布的topic
      _bboxes_pub = _nh.advertise<darknet_ros::bbox_array>("YOLO_boundingboxes", 1);//发布的topic

      cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
   }

   ~yoloObjectDetector()
   {
      cv::destroyWindow(OPENCV_WINDOW);
   }

private:
   void drawBBoxes(cv::Mat &input_frame, std::vector<PredBox> &class_boxes, int &class_obj_count,
		   cv::Scalar &bbox_color, const std::string &class_label)
   {
      darknet_ros::bbox bbox_result;//调用bbox消息类型，不要漏了头文件darknet_ros/bbox.h

      for (int i = 0; i < class_obj_count; i++)
      {
         int xmin = (class_boxes[i].x - class_boxes[i].w/2)*FRAME_W;
         int ymin = (class_boxes[i].y - class_boxes[i].h/2)*FRAME_H;
         int xmax = (class_boxes[i].x + class_boxes[i].w/2)*FRAME_W;
         int ymax = (class_boxes[i].y + class_boxes[i].h/2)*FRAME_H;

         bbox_result.Class = class_label;//bbox.msg类型
         bbox_result.prob = class_boxes[i].prob;//结合了另一个文档leggedrobotics文档
         bbox_result.xmin = xmin;
         bbox_result.ymin = ymin;
         bbox_result.xmax = xmax;
         bbox_result.ymax = ymax;
         _bbox_results_msg.bboxes.push_back(bbox_result);//bbox_array.msg类型

         // draw bounding box of first object found
         cv::Point topLeftCorner = cv::Point(xmin, ymin);
         cv::Point botRightCorner = cv::Point(xmax, ymax);
	 cv::rectangle(input_frame, topLeftCorner, botRightCorner, bbox_color, 2);
         cv::putText(input_frame, class_label, cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
		 1.0, bbox_color, 2.0);
      }
   }

   void runYOLO(cv::Mat &full_frame)
   {
      cv::Mat input_frame = full_frame.clone();
      std::vector< std::vector<PredBox> > class_bboxes(num_classes);
      std::vector<int> class_obj_count(num_classes, 0);

      // run yolo and get bounding boxes for objects
      _boxes = run_yolo();

      // get the number of bounding boxes found
      int num = get_obj_count(); //_boxes[0].num;

      // if at least one bbox found, draw box
      if (num > 0  && num <= 100)
      {
	 std::cout << "# Objects: " << num << std::endl;

	 // split bounding boxes by class
         for (int i = 0; i < num; i++)
         {
            for (int j = 0; j < num_classes; j++)
            {
               if (_boxes[i].Class == j)
               {
                  class_bboxes[j].push_back(_boxes[i]);
                  class_obj_count[j]++;
               }
            }
         }

	 // send message that an object has been detected
         std_msgs::Int8 msg;
         msg.data = 1;
         _found_object_pub.publish(msg);

         for (int i = 0; i < num_classes; i++)
         {
            if (class_obj_count[i] > 0) drawBBoxes(input_frame, class_bboxes[i],
					      class_obj_count[i], _bbox_colors[i], class_labels[i]);
         }
         _bboxes_pub.publish(_bbox_results_msg);
         _bbox_results_msg.bboxes.clear();
      }
      else
      {
          std_msgs::Int8 msg;
          msg.data = 0;
          _found_object_pub.publish(msg);
      }

      //for (int i = 0; i < num_classes; i++)
      //{
      //   _class_bboxes[i].clear();
      //   _class_obj_count[i] = 0;
      //}

      cv::imshow(OPENCV_WINDOW, input_frame);
      cv::waitKey(3);
   }
/*IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(input_image);//将Mat格式的转换成IplImage格式
   return ROS_img;
}
get_Ipl_image函数被run_yolo.cpp中的fetch_image函数调用,而fetch_image函数又被run_yolo函数调用,而run_yolo函数再被yolo_ros.cpp(即本.cpp文件)中的runYOLO函数调用,跟着又被imageCallback函数调用,然后在类yoloObjectDetector中调用了imageCallback,最后main调用了类yoloObjectDetector.
*/
   void imageCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      std::cout << "image received" << std::endl;

      cv_bridge::CvImagePtr cam_image;

      cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//sensor_msgs::image_encodings::BGR8看作一种类型

      if (cam_image)
      {
         input_image = cam_image->image.clone();
         runYOLO(cam_image->image);
      }
      return;
   }
};//类yoloObjectDetector到此结束

int main(int argc, char** argv)
{
   ros::init(argc, argv, "yolo_ros");

   ros::param::get(CAMERA_WIDTH_PARAM, FRAME_W);
   ros::param::get(CAMERA_HEIGHT_PARAM, FRAME_H);

   load_net(cfg, weights, thresh, 0.5);//配置cfg，权重
   yoloObjectDetector yuyv;//在类yoloObjectDetector中调用了imageCallback,imageCallback将usbcam获得的图像信息转化为IPImage格式的信息
   ros::spin();
   return 0;
}
