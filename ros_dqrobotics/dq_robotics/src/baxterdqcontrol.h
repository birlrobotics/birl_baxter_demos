/**
BAXTER Control Example with joint limits consideration in control loop.

\author Luis Figueredo 
\since 07/2014

*/


#ifndef BAXTER_DQ_CONTROL
#define BAXTER_DQ_CONTROL

#define MATH_PI 3.14159265
// #define DEFINE_BAXTER_REAL 1


#include "ros/ros.h"
#include <ros/console.h>
#include "../include/dq_robotics/DQ.h"
#include "../include/dq_robotics/DQ_kinematics.h"
#include "../include/dq_robotics/DQ_controller.h"
#include "../include/dq_robotics/DQ_CDTS.h"
#include "../include/dq_robotics/controllers/Controller_Generic.h"
#include "../include/dq_robotics/controllers/Controller_Cooperative.h"
	

#include "../include/dq_robotics/robot_dh/BaxterArm.h"

#include "baxter_core_msgs/DigitalIOState.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h" 
#include "std_msgs/String.h"

#include <std_srvs/Empty.h>
#include <sstream>
#include <vector>
#include <math.h> 
#include <string.h> 

#include "baxter_communication.h"





using namespace Eigen;
using namespace DQ_robotics;
 



int FLAG_CONTROLLER_OPTION = 0;


class baxterdqcontrol
{

public:	
	
private:

	// new ones
	ros::Subscriber  rosvar_sub__get_left_button, rosvar_sub__get_right_button;
	bool switching_time_button_left, switching_time_button_right;
	baxter_comm  *baxter_ros;
	double constlastjoint;
	double relativenewjoint;
	double relativeinitjoint_main;
	Matrix<double,7,1> relativeinit_refjoints; 	
	DQ relativeinit_dqmain;
	DQ relativerefpose;
	bool enable_fixed_joint_test;

	//*********   FLAGS  *************	
	bool FLAG_FEEDBACK_CONTROL_IN_CALLBACK;
	bool FLAG_PUBLISH_INFO_DQ;
	bool FLAG_PUBLISH_INFO_ERROR;
	bool FLAG_TRACK_ONLY_POSITION;
	bool FLAG__PRINT_DEBUG_INFORMATION;


	// // Simple counter and aux vars
	int i;

	//*********  ROS - MSGS PRINT/GET *************
	geometry_msgs::Pose 		    rosmsg_printpose, rosmsg_printrotation, rosmsg_getpose;
	geometry_msgs::Point 			rosmsg_getposition;
	std_msgs::String 				rosmsg_printerror;
	std::ostringstream 				rosaux_stringerror, rosaux_string;
	// 
	ros::ServiceServer 				rossrv_changeCtrl_idle, rossrv_changeCtrl_setpoint, rossrv_changeCtrl_tracking;


	// //*********  ROS ELEMENTS: ARM COMMUNICATION *************
	ros::NodeHandle rosnode;			  

	//*********  ROS ELEMENTS: PRINT/GET *************
	ros::Subscriber 	rosvar_subs__get_ref_pose,  	rosvar_subs__get_ref_position,  	rosvar_subs__get_ref_position_appended;
	ros::Publisher  	rosvar_pub__print_pose_left,    rosvar_pub__print_ERROR;


	//*********  DQ DATA *************
	DQ   	dqvar_dq_xd, 		dqvar_dq_xm, 	dqvar_dq_error;		
	DQ 		dqvar_dq_base;
	// 
	double dqvar_norm__dnum_ERROR; 				// Norm of the error
	Matrix<double,8,1> dqvar_temp__Mat8x1;		// Temp vec8
	double dqvar_temp__dvec4[4];

	//*********  REF DATA *************	
	ros::Publisher 	rosvar_pub__print_pose_right;
	geometry_msgs::Pose 			rosmsg_printpose_REF;	
	DQ 		dqvar_dq_refxm, 			dqvar_dq_relpose;
	DQ 		dqvar_dq_refbase, 			dqvar_dq_ref_initpose;
	DQ_kinematics 	dqvar_ARM_KINE_REF;	
	
	//*********  CONTROLLER DATA *************	
	int ctrlvar_robotnumberofdof;			
	double ctrlvar_pid__gain_kp, ctrlvar_pid__gain_ki, ctrlvar_pid__gain_kd;
	Matrix<double,8,8> ctrlvar_pid__Mat8x8_kp; 			
	Matrix<double,7,1> ctrlvar_joints__Mat7x1_output; 	
	// float ctrlvar_joints__fvec7_SpeedLimits[7];
	// 
	int ctrlvar_counter__ctrl_step;		// Discrete controller iteration counter
	double ctrlvar_norm__dnum_threshold;	// Pre-defined threshold for control error
	double ctrlvar_time__dnum_CONTROL_FREQ;


	//*********  ROBOT ARM DATA *************	
	std::string ROBOT_NAME;
	DQ_kinematics 	dqvar_ARM_KINE;			
	Matrix<double,7,1> ctrlvar_joints__Mat7x1_LIMITS_UPPER;
	Matrix<double,7,1> ctrlvar_joints__Mat7x1_LIMITS_LOWER;  


public:

	bool srvcallback_setctrl_idle(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	bool srvcallback_setctrl_setpoint(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
	bool srvcallback_setctrl_tracking(std_srvs::Empty::Request& request, std_srvs::Empty::Response &response);
	// bool srvcallback_setctrl_COOPERATIVE(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);


	DQ get_endeffector(const Matrix<double,7,1>& joints, bool main_arm);

	void callbackInput_leftbutton(const baxter_core_msgs::DigitalIOState& button_state);
	void callbackInput_rightbutton(const baxter_core_msgs::DigitalIOState& button_state);

	// 
	void callbackInputReferencePose(const geometry_msgs::Pose& inputREF);
	void callbackInputReferencePosition(const geometry_msgs::Point& inputREF);
	void callbackInputReferencePosition_append(const geometry_msgs::Point& inputREF);

	bool getConstantRelativePose(const Matrix<double,7,1>& argjoints);	

	void init_ROS_Config();
	void init_ROBOT_ARM_CONFIG();
	void execution();


	void publishinformation(DQ dq_state, bool printed);
	void publishinformation_ref(DQ dq_state, bool printed);
	void publishinfo_error();
	
	void init_setConstantValues(baxter_comm &baxtercommunication_data){
		constlastjoint=0;
		relativeinitjoint_main =0;
		relativenewjoint=0;
		enable_fixed_joint_test = false;


		// **********  SETTING BAXTER COMMUNICATION 
		baxter_ros = &baxtercommunication_data;

		// **************  BASES  ************** 
		dqvar_dq_base = DQ(0.92388, 0, 0,  0.38268,  -0.024803,   0.079139, 0.1074, 0.059879);  
		dqvar_dq_base = dqvar_dq_base*(dqvar_dq_base.norm().inv());	

		dqvar_dq_refbase = DQ(0.92388, 0, 0,  -0.38268,  0.024803,   0.079139, -0.1074, 0.059879);  		
		dqvar_dq_refbase = dqvar_dq_refbase*(dqvar_dq_refbase.norm().inv());	
		

		// **************  CONTROL GAIN ************** 
		ctrlvar_pid__gain_kp = 0.9;		
		ctrlvar_pid__gain_ki = 0.005;
		#ifdef FLAG_COMMUNICATION__REALBAXTER // DEFINE_BAXTER_REAL			
			// ctrlvar_pid__gain_kp = 0.15;
			// ctrlvar_pid__gain_ki = 0.0001;					
			ctrlvar_pid__gain_kp = 0.25;
			ctrlvar_pid__gain_ki = 0.0001;								
			std::cout << "[ REAL BAXTER ] Setting control gain to kp=" << ctrlvar_pid__gain_kp << "  and ki= " << ctrlvar_pid__gain_ki << std::endl;
		#endif 	

		#ifdef FLAG_COMMUNICATION__REALBAXTER
			std::cout << " [ REAL BAXTER ] " << std::endl;
			sleep(1);
		#endif	
		#ifndef FLAG_COMMUNICATION__REALBAXTER
			std::cout << " [ GAZEBO BAXTER ] " << std::endl;
			sleep(1);
		#endif				
		ctrlvar_pid__gain_kd = 0.05;
		ctrlvar_time__dnum_CONTROL_FREQ = 200;   // CONTROL FREQUENCE: 200Hz
		// ctrlvar_time__dnum_CONTROL_FREQ = 125;

		// ************** INFO FLAGS ************** 
		FLAG_TRACK_ONLY_POSITION = false;
		FLAG_FEEDBACK_CONTROL_IN_CALLBACK = false;
		// Flags for publishing data about the manipulator
		FLAG_PUBLISH_INFO_ERROR = true;		
		FLAG_PUBLISH_INFO_DQ = true;
		// Print errors and dq pose information
		FLAG__PRINT_DEBUG_INFORMATION =  true;

	}

private:
};





#endif 
// BAXTER_DQ_CONTROL

