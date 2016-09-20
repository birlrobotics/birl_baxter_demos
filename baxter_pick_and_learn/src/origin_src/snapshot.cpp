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
#include <baxter_core_msgs/OpenCamera.h>
#include <baxter_core_msgs/CloseCamera.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

char *filename;

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image acquisition failed: %s", e.what());
        return;
    }
    cv::imwrite(filename, cv_ptr->image);
    ros::shutdown();
}

int main(int argc, char **argv)
{
    if(argc<2)
        std::cout << "Please specify a filename to save the image to." << std::endl;
    filename = argv[1];
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    baxter_core_msgs::OpenCamera open_camera;
    // Open the camera
    std::cout << "Opening right hand camera: ";
    open_camera.request.name = "right_hand_camera";
    open_camera.request.settings.width = 1280;
    open_camera.request.settings.height = 800;
    open_camera.request.settings.fps = 20;
    if(!ros::service::call("/cameras/open", open_camera) || open_camera.response.err != 0)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return 1;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;
    // Register the camera callback that receive the images
    std::cout << "Registrating camera callback: ";
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber is;
    is = it.subscribe("/cameras/right_hand_camera/image", 1, cameraCallback);
    if(is == NULL)
    {
        std::cout << "\033[1;31mFailed\033[0m" << std::endl;
        return 2;
    }
    std::cout << "\033[1;32mOK\033[0m" << std::endl;

    ros::spin();
    return 0;
}
