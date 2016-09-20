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

#include "camera.h"

Camera::Camera(Camera::CameraId id, ros::NodeHandle nh, std::vector<Piece> &highlight) :
pieces(highlight)
{
    hmin = 0;
    hmax = 100;
    vmin = 30;
    threshold1 = 20;
    threshold2 = 40;
    minarea = 1000;
    request_status = STATUS_AVAILABLE;
    this->nh = nh;
    // Open the camera
    std::cout << std::setw(80) << std::left << "Opening right hand camera: ";
    if(id == LEFT_HAND)
        open_camera.request.name = "left_hand_camera";
    else if(id == HEAD)
        open_camera.request.name = "head_camera";
    else if(id == RIGHT_HAND)
        open_camera.request.name = "right_hand_camera";
    else
        return;
    open_camera.request.settings.width = 1280;
    open_camera.request.settings.height = 800;
    open_camera.request.settings.fps = 20;
    camera_control.id = baxter_core_msgs::CameraControl::CAMERA_CONTROL_GAIN;
    camera_control.value = 10;
    open_camera.request.settings.controls.push_back(camera_control);
    if(!ros::service::call("/cameras/open", open_camera) || open_camera.response.err != 0)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    // Register the camera callback that receive the images
    std::cout << std::setw(80) << std::left << "Registering camera callback: ";
    it = new image_transport::ImageTransport(nh);
    is = it->subscribe("/cameras/right_hand_camera/image", 1, &Camera::callback, this);
    if(is == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    resetAim();
}

Camera::~Camera()
{
    delete it;
}

void Camera::callback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_RGB2HSV);
    img_gray.create(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    for(int i = 0; i < img_hsv.total(); i++)
    {
        if(img_hsv.data[i*3]<hmin || img_hsv.data[i*3]>hmax || img_hsv.data[i*3+2]<vmin)
            img_gray.data[i] = 0;
        else
            img_gray.data[i] = img_hsv.data[i*3+2];
    }
    cv::blur(img_gray, img_gray, cv::Size(5,5));
    cv::Canny(img_gray, img_gray, threshold1, threshold2);
    cv::dilate(img_gray, img_gray, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3)));
    cv::findContours(img_gray, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(request_status == STATUS_REQUESTING)
    {
       request_result = new std::vector<std::vector<cv::Point> >();
       request_status = STATUS_IN_PROGRESS;
    }
    for(int i = 0; i<contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if(area>minarea)
        {
            cv::approxPolyDP(contours[i], contours[i], sqrt(area)/20.0, true);
            int match = closestMatch(pieces, contours[i], 0.05);
            if(match != -1)
            {
                cv::drawContours(cv_ptr->image, contours, i, cv::Scalar(0,255,0),3, CV_AA);
                cv::putText(cv_ptr->image, pieces[match].getName(), contours[i][0], cv::FONT_HERSHEY_SIMPLEX, 2.0, cv::Scalar(0,255,0), 3, CV_AA);
            }
            else
                cv::drawContours(cv_ptr->image, contours, i, cv::Scalar(255,0,0),3, CV_AA);
            if(request_status == STATUS_IN_PROGRESS)
            {
                if(request_type == REQUEST_SELECTED_SHAPE)
                {
                    if(cv::pointPolygonTest(contours[i], cv::Point(666, 240), false) != -1)
                        request_result->push_back(std::vector<cv::Point>(contours[i]));
                }
                else if(request_type == REQUEST_SHAPES)
                    request_result->push_back(std::vector<cv::Point>(contours[i]));
            }
            cv::line(cv_ptr->image, cv::Point(aim_x - 10, aim_y), cv::Point(aim_x+10, aim_y), cv::Scalar(0,0,255), 3, CV_AA);
            cv::line(cv_ptr->image, cv::Point(aim_x, aim_y-10), cv::Point(aim_x, aim_y+10), cv::Scalar(0,0,255), 3, CV_AA);
        }
    }
    if(request_status == STATUS_IN_PROGRESS)
        request_status = STATUS_AVAILABLE;
    cv::imshow("Baxter Pick And Learn", cv_ptr->image);
    cv::waitKey(3);
}

void Camera::setAim(int x, int y)
{
    aim_x = x;
    aim_y = y;
}

void Camera::resetAim()
{
    aim_x = 666;
    aim_y = 240;
}

void Camera::request(Camera::RequestType request_type)
{
    if(request_status == STATUS_AVAILABLE)
    {
        this->request_type = request_type;
        request_status = STATUS_REQUESTING;
    }
}

bool Camera::isResultAvailable()
{
    if(request_status == STATUS_AVAILABLE)
        return true;
    else
        return false;
}


std::vector<std::vector<cv::Point> >* Camera::getResult()
{
    if(request_status == STATUS_AVAILABLE)
        return request_result;
    else
        return NULL;
}


void Camera::cameraTransform(float &x, float &y, float dz)
{
    /*
    h = 14.5 cm
    x = 50 cm
    y = 28 cm
    */
    x = (x/640)*0.22*(dz/0.145);
    y = (y/400)*0.12*(dz/0.145);
}

int Camera::getClosestMatchApproach(std::vector<std::vector<cv::Point> > *result, std::vector<Piece> &pieces, float z, float obj_position[], float obj_orientation[], int &match)
{
        int matchs, matchp;
        float dz, x, y;
        closestMatch(pieces, (*result), 0.1, matchp, matchs);
        if(matchs == -1 || matchp == -1)
            return 1;
        else
        {
            cv::Moments m1 = cv::moments((*result)[matchs]);
            cv::Moments m2 = pieces[matchp].getMoments();
            x = m1.m10/m1.m00;
            y = m1.m01/m1.m00;
            dz = z-pieces[matchp].getPickingHeight();
            setAim((int) x, (int) y);
            x -= m2.m10/m2.m00;
            y -= m2.m01/m2.m00;
            cameraTransform(x, y, dz+0.08);
            obj_position[0] = y;
            obj_position[1] = x;
            obj_position[2] = -dz;
            pieces[matchp].getPickingOrientation(obj_orientation);
            match = matchp;
            return 0;
        }
}

