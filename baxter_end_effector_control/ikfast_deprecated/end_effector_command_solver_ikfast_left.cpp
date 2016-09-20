#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IKREAL_TYPE IkReal // for IKFast 56,61

#include "baxter_left_arm_ikfast_solver.cpp"
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <string>

using namespace geometry_msgs;
const std::string limbName = "left";
// the joint order of solution is s -> e -> w, while in joint state is e -> s -> w
const unsigned int leftJointAngleIndex[] = {10, 11, 8, 9, 12, 13, 14};

ros::Publisher jointCommandPublisher;
sensor_msgs::JointState currentJointState;

bool findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz, IkSolutionList<IKREAL_TYPE> &solutions);
void publishSolution(const IkSolutionBase<IKREAL_TYPE>& solution);

// only call this function if solutions is not empty
const IkSolutionBase<IKREAL_TYPE>& selectBestSolution(const IkSolutionList<IKREAL_TYPE> &solutions);
IKREAL_TYPE getEstimatedCostFromCurrentJointState(const IkSolutionBase<IKREAL_TYPE>& target);

void callBack(const PoseStamped &target) {
    ROS_INFO("recieved left arm command: %lf %lf %lf %lf %lf %lf %lf", target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z);
    
    IkSolutionList<IKREAL_TYPE> solutions;
    bool bSuccess = findIKSolutions(target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, solutions);
    
    if (bSuccess) {
        publishSolution(selectBestSolution(solutions));
    }
}

void jointStateCallBack(const sensor_msgs::JointState &target) {
    //ROS_INFO_STREAM("receive JointState");
    currentJointState = target;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_command_solver_ikfast_left");
    ros::NodeHandle n;
    jointCommandPublisher = n.advertise<baxter_core_msgs::JointCommand>("end_effector_command_solution", 1);
    ros:: Subscriber commandPositionSubscriber = n.subscribe("end_effector_left_end_command_position", 1, callBack);
    ros:: Subscriber jointStateSubscriber = n.subscribe("robot/joint_states", 1, jointStateCallBack);
    ROS_INFO_STREAM("end_effector_command_solver_ikfast_left subscribing...");
    ros::spin();
    return 0;
}

IKREAL_TYPE getEstimatedCostFromCurrentJointState(const IkSolutionBase<IKREAL_TYPE>& target) {
    const unsigned int num_of_joints = GetNumJoints();
    
    std::vector<IKREAL_TYPE> solvalues(num_of_joints);
    
    int this_sol_free_params = (int)target.GetFree().size(); 
    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);
    target.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    
    IKREAL_TYPE result = 0;
    IKREAL_TYPE temp = 0;
    for (unsigned int count = 0; count < num_of_joints; count++) {
        temp = solvalues[count] - currentJointState.position[leftJointAngleIndex[count]];
        result += temp * temp;
    }
    
    return result;
}

// only call this function if solutions is not empty
const IkSolutionBase<IKREAL_TYPE>& selectBestSolution(const IkSolutionList<IKREAL_TYPE> &solutions) {
    const unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
    
    unsigned int resultIndex = 0;
    IKREAL_TYPE minCost = getEstimatedCostFromCurrentJointState(solutions.GetSolution(0));
    IKREAL_TYPE currentCost = 0;
    for (unsigned int count = 1; count < num_of_solutions; count++) {
        currentCost = getEstimatedCostFromCurrentJointState(solutions.GetSolution(count));
        if (minCost > currentCost) {
            resultIndex = count;
        }
    }
    ROS_INFO("Select solution in %d", resultIndex);
    printf("Current joint angle is :");
    const unsigned int num_of_joints = GetNumJoints();
    for (unsigned int count = 0; count < num_of_joints; count++) {
        printf(" %lf", currentJointState.position[leftJointAngleIndex[count]]);
    }
    printf("\n");
    return solutions.GetSolution(resultIndex);
}

void publishSolution(const IkSolutionBase<IKREAL_TYPE>& solution) {
    unsigned int num_of_joints = GetNumJoints();
    
    std::vector<IKREAL_TYPE> solvalues(num_of_joints);

    baxter_core_msgs::JointCommand command;
    command.mode = 1;
    std::string jointNamesArray[] = {limbName + "_s0", limbName + "_s1", limbName + "_e0", limbName + "_e1",
        limbName + "_w0", limbName + "_w1", limbName + "_w2"};
    std::vector<std::string> jointNamesVector(num_of_joints);
    for( std::size_t j = 0; j < num_of_joints; ++j)
        jointNamesVector[j] = jointNamesArray[j];
    command.names = jointNamesVector;

    int this_sol_free_params = (int)solution.GetFree().size(); 
    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

    solution.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    command.command.resize(num_of_joints);
    printf("JointCommand to be published: ");
    for( std::size_t j = 0; j < num_of_joints; ++j) {
        command.command[j] = solvalues[j];
        printf(" %lf", solvalues[j]);
    }
    printf("\n");
    
    jointCommandPublisher.publish(command);
}

bool findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz, IkSolutionList<IKREAL_TYPE> &solutions) {
    IKREAL_TYPE eerot[9],eetrans[3];
    unsigned int num_free_parameters = GetNumFreeParameters();
    
    eetrans[0] = _tx;
    eetrans[1] = _ty;
    eetrans[2] = _tz;

    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
    // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
    double qw = _qw;
    double qx = _qx;
    double qy = _qy;
    double qz = _qz;
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    const int w1Index = 13;
    IKREAL_TYPE currentW1Angle = currentJointState.position[w1Index];
    IKREAL_TYPE vfree = 0;
    ROS_INFO_STREAM("current w1 angle:");
    ROS_INFO_STREAM(currentW1Angle);
    ROS_INFO_STREAM("success in:");
    IKREAL_TYPE currentFactor = 0;
    
    bool bSuccess = false;
    do {
        if (currentW1Angle + currentFactor < 2.094) {
            vfree = currentW1Angle + currentFactor;
            bSuccess = ComputeIk(eetrans, eerot, &vfree, solutions);
            if (bSuccess) {
                ROS_INFO_STREAM(vfree);
                break;
            }
        }
        
        if (currentW1Angle - currentFactor > -1.571) {
            vfree = currentW1Angle - currentFactor;
            bSuccess = ComputeIk(eetrans, eerot, &vfree, solutions);
            if (bSuccess) {
                ROS_INFO_STREAM(vfree);
                break;
            }
        }
        
        if(!bSuccess) {
            currentFactor += 0.01;
        }
        if (currentW1Angle + currentFactor > 2.094 && currentW1Angle - currentFactor < -1.571) {
            printf("Exceeded max w1 angle, returning...\n");
            break;
        }
    } while (!bSuccess);
    
    ROS_INFO_STREAM("Found ik solutions count:");
    ROS_INFO_STREAM(solutions.GetNumSolutions());
    return bSuccess;
}
