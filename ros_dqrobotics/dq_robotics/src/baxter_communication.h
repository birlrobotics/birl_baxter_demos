/**
BAXTER COMMUNICATION PROGRAM

\author Luis Figueredo 
\since 07/2014

*/


#ifndef DQ_BAXTER_COMMUNICATION
#define DQ_BAXTER_COMMUNICATION

// UNCOMMENT THIS LINE TO USE IN REAL BAXTER!
#define FLAG_COMMUNICATION__REALBAXTER 1


#define MATH_PI 3.14159265

#include "ros/ros.h"
#include <ros/console.h>

// ROS Messages
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include "trajectory_msgs/JointTrajectory.h"
// ****[ luis-quadcopter ]
#ifdef FLAG_COMMUNICATION__REALBAXTER
	#include "baxter_core_msgs/JointCommand.h"
#endif 

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h" 
#include "std_msgs/String.h"
#include <string.h> 
#include <Eigen/Dense>





class baxter_comm
{

public:

	Eigen::Matrix<double,7,1>  left_joints;
	Eigen::Matrix<double,7,1>  right_joints;
	Eigen::Matrix<double,14,1>  left_right_joints;

	bool FLAG_REAL_BAXTER_COMMUNICATION;
	bool FLAG_INIT__READING_LEFT_JOINTS;
	bool FLAG_INIT__READING_RIGHT_JOINTS;

private:


	ros::NodeHandle rosnode;			  

	//**************** ROS - MSGS
	sensor_msgs::JointState 				rosvar_msg__input;
	#ifdef FLAG_COMMUNICATION__REALBAXTER
		baxter_core_msgs::JointCommand	    rosmsg_output; 
	#endif 		
	#ifndef FLAG_COMMUNICATION__REALBAXTER
		trajectory_msgs::JointTrajectory			rosmsg_output; 
	#endif 		
	trajectory_msgs::JointTrajectoryPoint		rosmsg_trajectPoints; 
	trajectory_msgs::JointTrajectoryPoint		rosmsg_trajectPoints_vec14; 		

	//**************** ROS - TOPICS
	ros::Subscriber rossub_input_getjoints;	
	ros::Publisher 	rospub_output_leftarm_joints, rospub_output_rightarm_joints;	

	//**************** SEQUENCE OF JOINT NAMES (AND INDEX)
	std::vector<std::string> left_right__joint_names;	
	std::vector<std::string> left__joint_names;	
	std::vector<std::string> right_joint_names;	
	int left__joint_names_Seq7[7];
	int right_joint_names_Seq7[7];	

	//**************** JOINT LIMITS (NOT USED HERE)
	Eigen::Matrix<double,7,1> CONST_JOINTS_LIMITS_UPPER;
	Eigen::Matrix<double,7,1> CONST_JOINTS_LIMITS_LOWER;  

	//**************** Speed limits
	float speedlimits_left[7];
	float speedlimits_right[7];
	
	// Simple counter and aux vars
	int loopcb, loopcb_index;	
	int rosmsg_output_counter;


public:

	baxter_comm(){};
	baxter_comm(int argc, char **argv);
	~baxter_comm(){};

	void publish_left_joints(const Eigen::Matrix<double,7,1>& outputJoints);	
	void publish_right_joints(const Eigen::Matrix<double,7,1>& outputJoints);	
	void publish_leftright_joints(const Eigen::Matrix<double,14,1>& outputJoints);	



private:

	void publish_to_arm(const Eigen::Matrix<double,7,1>& outputJoints, bool left_arm);
	void callback_getJoints(const sensor_msgs::JointState& sensordata);	

	inline float dqsgn(const float x);
	inline float dqlimspeed(const float xnew, const float x0, const float maxspeed);
};





#endif 
// DQBAXTERARM







// DQ_BAXTER_COMMUNICATION
