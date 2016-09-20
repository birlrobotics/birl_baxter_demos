#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast
#define IKREAL_TYPE IkReal // for IKFast 56,61

#include "baxter_right_arm_ikfast_solver.cpp"
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <string>

using namespace geometry_msgs;

const std::string limbName = "right";

ros::Publisher jointCommandPublisher;

bool findIKSolutions(double _tx, double _ty, double _tz, double _qw, double _qx, double _qy, double _qz, IkSolutionList<IKREAL_TYPE> &solutions);
void publishSolution(const IkSolutionList<IKREAL_TYPE> &solutions);

void callBack(const PoseStamped &target) {
    ROS_INFO("recieved right arm command: %lf %lf %lf %lf %lf %lf %lf", target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z);
    
    IkSolutionList<IKREAL_TYPE> solutions;
    bool bSuccess = findIKSolutions(target.pose.position.x, target.pose.position.y, target.pose.position.z,
        target.pose.orientation.w, target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, solutions);
    
    if (bSuccess) {
        publishSolution(solutions);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "end_effector_command_solver_ikfast_right");
    ros::NodeHandle n;
    jointCommandPublisher = n.advertise<baxter_core_msgs::JointCommand>("end_effector_command_solution", 1);
    ros:: Subscriber s = n.subscribe("end_effector_right_end_command_position", 1, callBack);
    ROS_INFO_STREAM("end_effector_command_solver_ikfast_right subscribing...");
    ros::spin();
    return 0;
}

void publishSolution(const IkSolutionList<IKREAL_TYPE> &solutions) {
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
    printf("Found %d ik solutions:\n", num_of_solutions ); 
    std::vector<IKREAL_TYPE> solvalues(num_of_joints);

    baxter_core_msgs::JointCommand command;
    command.mode = 1;
    std::string jointNamesArray[] = {limbName + "_s0", limbName + "_s1", limbName + "_e0", limbName + "_e1",
        limbName + "_w0", limbName + "_w1", limbName + "_w2"};
    std::vector<std::string> jointNamesVector(num_of_joints);
    for( std::size_t j = 0; j < num_of_joints; ++j)
        jointNamesVector[j] = jointNamesArray[j];
    command.names = jointNamesVector;

    const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(0);
    int this_sol_free_params = (int)sol.GetFree().size(); 
    std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

    sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    command.command.resize(num_of_joints);
    for( std::size_t j = 0; j < num_of_joints; ++j)
        command.command[j] = solvalues[j];
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

    IKREAL_TYPE vfree = -1.571;

    bool bSuccess = false;
    do {
        bSuccess = ComputeIk(eetrans, eerot, &vfree, solutions);
        if(!bSuccess) {
            vfree += 0.1;
        }
        if (vfree > 2.094) {
            printf("Exceeded max w1 angle, returning...\n");
            break;
        }
    } while (!bSuccess);
    
    return bSuccess;
}
