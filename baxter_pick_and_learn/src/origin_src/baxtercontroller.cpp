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

#include "baxtercontroller.h"

BaxterController::BaxterController(ros::NodeHandle nh)
{
    input = INPUT_NOTHING;
    gripper_hid = 0;
    has_to_move = false;
    last_input_time = clock();
    this->nh = nh;
    std::cout << std::setw(80) << std::left << "Registering ITB callbacks: ";
    itb_sub = nh.subscribe("/robot/itb/right_itb/state", 2, &BaxterController::itbCallback, this);
    if(itb_sub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering gripper callback: ";
    gripper_sub = nh.subscribe("/robot/end_effector/right_gripper/state", 2, &BaxterController::gripperCallback, this);
    if(gripper_sub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering gripper publisher: ";
    gripper_pub = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", 2);
    if(gripper_pub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering IR subscriber: ";
    ir_sub = nh.subscribe("/robot/range/right_hand_range/state", 2, &BaxterController::irCallback, this);
    if(ir_sub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering endpoint subscriber: ";
    endpoint_sub = nh.subscribe("/robot/limb/right/endpoint_state", 2, &BaxterController::endpointCallback, this);
    if(endpoint_sub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering inverse kinematic solver client: ";
    ik_client = nh.serviceClient<baxter_core_msgs::SolvePositionIK>("ExternalTools/right/PositionKinematicsNode/IKService");
    if(ik_client == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
    std::cout << std::setw(80) << std::left << "Registering joint publisher: ";
    joint_pub = nh.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command", 2);
    if(joint_pub == NULL)
    {
        std::cout << std::right << "\033[1;31m[Failed]\033[0m" << std::endl;
        return;
    }
    std::cout << std::right << "\033[1;32m[OK]\033[0m" << std::endl;
}

BaxterController::~BaxterController()
{
}

void BaxterController::itbCallback(const baxter_core_msgs::ITBStateConstPtr &msg)
{
    if(clock() - last_input_time > INPUT_BLOCKING_TIME)
    {
        if(msg->buttons[0])
        {
            input = INPUT_WHEEL_CLICKED;
            last_input_time = clock();
        }
        else if(msg->buttons[1])
        {
            input = INPUT_BACK_CLICKED;
            last_input_time = clock();
        }
        else if(msg->buttons[2])
        {
            input = INPUT_HOME_CLICKED;
            last_input_time = clock();
        }
    }
}

void BaxterController::gripperCallback(const baxter_core_msgs::EndEffectorStateConstPtr &msg)
{
    if(gripper_hid == 0)
    {
        gripper_hid = msg->id;
        baxter_core_msgs::EndEffectorCommand cmd;
        cmd.id = gripper_hid;
        cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
        gripper_pub.publish(cmd);
    }
}

void BaxterController::irCallback(const sensor_msgs::RangeConstPtr &msg)
{
    this->range = msg->range;
}

void BaxterController::endpointCallback(const baxter_core_msgs::EndpointStateConstPtr &msg)
{
    position[0] = msg->pose.position.x;
    position[1] = msg->pose.position.y;
    position[2] = msg->pose.position.z;

    orientation[0] = msg->pose.orientation.x;
    orientation[1] = msg->pose.orientation.y;
    orientation[2] = msg->pose.orientation.z;
    orientation[3] = msg->pose.orientation.w;
    if(has_to_move)
        joint_pub.publish(joint_cmd);
}

float BaxterController::getRange()
{
    return range;
}

void BaxterController::getPosition(float position[])
{
    for(int i = 0; i < 3; i++)
        position[i] = this->position[i];
}

void BaxterController::getOrientation(float orientation[])
{
    for(int i = 0; i < 4; i++)
        orientation[i] = this->orientation[i];
}

BaxterController::ITBInput BaxterController::getInput()
{
    ITBInput t = input;
    input = INPUT_NOTHING;
    return t;
}

void BaxterController::grip()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_GRIP;
    gripper_pub.publish(cmd);
}

void BaxterController::release()
{
    baxter_core_msgs::EndEffectorCommand cmd;
    cmd.id = gripper_hid;
    cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_RELEASE;
    gripper_pub.publish(cmd);
}

int BaxterController::moveTo(float position[], float orientation[])
{
    baxter_core_msgs::SolvePositionIK srv;
    geometry_msgs::PoseStamped pose_stamped;
    set_position[0] = position[0];
    set_position[1] = position[1];
    set_position[2] = position[2];
    set_orientation[0] = orientation[0];
    set_orientation[1] = orientation[1];
    set_orientation[2] = orientation[2];
    set_orientation[3] = orientation[3];
    pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "base";
    pose_stamped.pose.position.x = position[0];
    pose_stamped.pose.position.y = position[1];
    pose_stamped.pose.position.z = position[2];
    pose_stamped.pose.orientation.x = orientation[0];
    pose_stamped.pose.orientation.y = orientation[1];
    pose_stamped.pose.orientation.z = orientation[2];
    pose_stamped.pose.orientation.w = orientation[3];
    srv.request.pose_stamp.push_back(geometry_msgs::PoseStamped());
    srv.request.pose_stamp.push_back(pose_stamped);
    if(!ik_client.call(srv))
    {
       std::cout << "\033[1;31mCall to inverse kinematic solver service failed\033[0m" << std::endl;
       return 1;
    }
    if(!srv.response.isValid[1])
    {
       std::cout << "\033[1;31mInverse kinematic solver found no solution for that movement\033[0m" << std::endl;
       return 1;
    }
    has_to_move = false;
    joint_cmd.mode =  baxter_core_msgs::JointCommand::POSITION_MODE;
    joint_cmd.names = srv.response.joints[1].name;
    joint_cmd.command = srv.response.joints[1].position;
    has_to_move = true;
    return 0;
}

int BaxterController::move(float position[], float orientation[])
{
    float p[3];
    for(int i = 0; i<3; i++)
        p[i] = this->position[i] + position[i];
    return moveTo(p, orientation);
}   

void BaxterController::stop()
{
    has_to_move = false;
}

float BaxterController::distanceToSetPosition()
{
    float d = 0;
    for(int i = 0; i < 3; i++)
        d += (position[i]-set_position[i])*(position[i]-set_position[i]);
    for(int i = 0; i < 4; i++)
        d += (orientation[i]-set_orientation[i])*(orientation[i]-set_orientation[i]);
    return sqrt(d);
}
