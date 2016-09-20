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
#include <vector>
#include <math.h>
#include "baxtercontroller.h"
#include "camera.h"

#define MAX_PASS 4

int main(int argc, char **argv)
{
    // Initialise ROS
    ros::init(argc, argv, "baxter_pickandlearn");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    std::vector<Piece> pieces;
    BaxterController *robot = new BaxterController(nh);
    Camera  *camera =  new Camera(Camera::RIGHT_HAND, nh, pieces);
    int state = 0;
    float obs_position[3] = {0.55, -0.4, 0.15};
    float obs_orientation[4] = {1,0,0,0};
    float position[3], obj_position[3], orientation[4], obj_orientation[4];
    spinner.start();
    // Learning
    bool learning = true;
    robot->release();
    while(learning)
    {
        BaxterController::ITBInput input = robot->getInput();
        if(input == BaxterController::INPUT_HOME_CLICKED)
            learning = false;
        if(state == 0)
        {
            if(input ==  BaxterController::INPUT_WHEEL_CLICKED)
            {
                camera->request(Camera::REQUEST_SELECTED_SHAPE);
                state = 1;
            }
        }
        else if(state == 1 && camera->isResultAvailable())
        {
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
            {
                std::cout << "No result" << std::endl;
                state = 0;
            }
            else
            {
                state = 2;
                robot->getPosition(position);
                robot->getOrientation(orientation);
                char buffer[32];
                snprintf(buffer, 32, "Piece %d", pieces.size()+1);
                pieces.push_back(Piece((*result)[0], position[2], orientation, std::string(buffer)));
                std::cout << pieces.back().getName() << " saved:" << std::endl;
                robot->grip();
            }
            delete result;
        }
        else if(state == 2 && input == BaxterController::INPUT_WHEEL_CLICKED)
        {
            robot->getPosition(position);
            robot->getOrientation(orientation);
            pieces.back().setDropPosition(position, orientation);
            state = 0;
            robot->release();
        }
        ros::Duration(0.1).sleep();
    }
    bool sorting = true;
    state = 0;
    while(sorting)
    {
        int pass, match;
        BaxterController::ITBInput input = robot->getInput();
        if(input == BaxterController::INPUT_HOME_CLICKED)
            sorting = false;
        if(state == 0)
        {
            robot->release();
            robot->moveTo(obs_position, obs_orientation);
            state = 1;
        }
        else if(state == 1)
        {
            if(robot->distanceToSetPosition() < 0.01)
            {
                pass = 1;
                state = 2;
            }
        }
        else if(state == 2)
        {
            camera->request(Camera::REQUEST_SHAPES);
            state = 3;
        }
        else if(state == 3 && camera->isResultAvailable())
        {
            std::vector<std::vector<cv::Point> > *result = camera->getResult();
            if(result->size() == 0 || (*result)[0].size() == 0)
            {
                std::cout << "No shape found" << std::endl;
                state = 2;
            }
            else
            {
                robot->getPosition(position);
                if(camera->getClosestMatchApproach(result, pieces, position[2], obj_position, obj_orientation, match))
                {
                    std::cout << "No piece found" << std::endl;
                    state = 2;
                }
                else
                {
                    for(int i = 0; i < 3; i++)
                        obj_position[i] += position[i];
                    std::cout << "Found at x: " << obj_position[0] << " y: " << obj_position[1] << " z: " << obj_position[2] << std::endl;
                    if(pass == MAX_PASS)
                    {
                        if(robot->moveTo(obj_position, obj_orientation))
                            state = 2;
                        else
                            state = 4;
                    }
                    else
                    {
                        // Smooth approach
                        obj_position[0] -= 0.1 - 0.08*pass/MAX_PASS;
                        obj_position[2] += 0.1 - 0.08*pass/MAX_PASS;
                        if(robot->moveTo(obj_position, obs_orientation))
                            state = 2;
                        else
                            state = 4;
                    }
                }
            }
            delete result;
        }
        else if(state == 4)
        {
            if(robot->distanceToSetPosition() < 0.01)
            {
                pass++;
                if(pass<=MAX_PASS)
                    state = 2;
                else
                {
                    state = 5;
                    if(robot->moveTo(obj_position, obj_orientation))
                        state = 0;
                }
            }
        }
        else if(state == 5)
        {
            if(robot->distanceToSetPosition() < 0.01)
            {
                robot->grip();
                ros::Duration(0.5).sleep();
                obj_position[2] += 0.2;
                state = 6;
                if(robot->moveTo(obj_position, obj_orientation))
                    state = 0;
            }
        }
        else if(state == 6)
        {
            if(robot->distanceToSetPosition() < 0.01)
            {
                state = 7;
                pieces[match].getDropPosition(obj_position);
                pieces[match].getDropOrientation(obj_orientation);
                if(robot->moveTo(obj_position, obj_orientation))
                    state = 0;
            }
        }
        else if (state == 7)
        {
            if(robot->distanceToSetPosition() < 0.01)
            {
                robot->release();
                state = 0;
            }
        }
        ros::Duration(0.1).sleep();
    }
    spinner.stop();
    delete robot;
    delete camera;
    return 0;
}
