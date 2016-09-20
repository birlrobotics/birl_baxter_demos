/*
This file is part of Baxter Pick and Learn.

Baxter Pick and Learn is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Nagen is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

Copyright 2014 Charles Hubain <charles.hubain@haxelion.eu>
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <baxter_core_msgs/CameraSettings.h>
#include <baxter_core_msgs/CameraControl.h>
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <math.h>
#include "piece.h"


class Camera
{
public:
    enum CameraId {LEFT_HAND, HEAD, RIGHT_HAND};
    enum RequestType {REQUEST_SELECTED_SHAPE, REQUEST_SHAPES};
    enum RequestStatus {STATUS_AVAILABLE, STATUS_REQUESTING, STATUS_IN_PROGRESS};

    Camera(CameraId id, ros::NodeHandle nh, std::vector<Piece> &highlight);
    ~Camera();
    void callback(const sensor_msgs::ImageConstPtr &msg);
    void request(RequestType request_type);
    void setAim(int x, int y);
    void resetAim();
    void cameraTransform(float &x, float &y, float dz);
    bool isResultAvailable();
    std::vector<std::vector<cv::Point> >* getResult();
    int getClosestMatchApproach(std::vector<std::vector<cv::Point> > *result, std::vector<Piece> &pieces, float z, float obj_position[], float obj_orientation[], int &match);

private:
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat img_gray, img_hsv;
    ros::NodeHandle nh;
    baxter_core_msgs::OpenCamera open_camera;
    baxter_core_msgs::CameraControl camera_control;
    image_transport::ImageTransport *it;
    image_transport::Subscriber is;
    std::vector<std::vector<cv::Point> > contours, *request_result;
    std::vector<Piece> &pieces;
    int hmin, hmax, vmin, threshold1, threshold2, minarea;
    int aim_x, aim_y;
    RequestType request_type;
    RequestStatus request_status;
};
