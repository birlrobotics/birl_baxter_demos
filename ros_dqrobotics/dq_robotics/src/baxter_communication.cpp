/**
BAXTER COMMUNICATION PROGRAM

\author Luis Figueredo 
\since 07/2014

*/


#include "baxter_communication.h"


//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : baxter_comm           #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  

/// * CALL init_default_parameters with robot input (DQ_kinematics)
baxter_comm::baxter_comm(int argc, char **argv)
{    	

	//**************************************************[   ROS  ]	

		// *****  INIT ROS
		// *****************************************
		ros::init(argc, argv, "dq_baxter_communication",  ros::init_options::AnonymousName);
		// ros::init(std::map<std::string, std::string>, std::vector<std::pair<std::string, std::string> >, "dq_baxter_communication",  ros::init_options::AnonymousName);



		std::cout << std::endl;
		ROS_INFO("[ Baxter-Comm ] Starting Communication with Baxter");   

		// *****  Adverstise and Subscribe
		// *****************************************
		#ifdef FLAG_COMMUNICATION__REALBAXTER
			rospub_output_leftarm_joints  = rosnode.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1000);	
			rospub_output_rightarm_joints = rosnode.advertise<baxter_core_msgs::JointCommand>("/robot/limb/right/joint_command",1000);	
		#endif 
		#ifndef FLAG_COMMUNICATION__REALBAXTER		
			rospub_output_leftarm_joints = rosnode.advertise<trajectory_msgs::JointTrajectory>("/robot/left_position_trajectory_controller/command",1000);	
			rospub_output_rightarm_joints = rosnode.advertise<trajectory_msgs::JointTrajectory>("/robot/right_position_trajectory_controller/command",1000);	
		#endif 		
		rossub_input_getjoints   = rosnode.subscribe("/robot/joint_states",1000, &baxter_comm::callback_getJoints, this);		

		// *****  Set Msgs 
		// *****************************************
		rosmsg_trajectPoints.positions.resize(7);	
		#ifndef FLAG_COMMUNICATION__REALBAXTER			
			rosmsg_output.points.resize(1);
		#endif 	
		rosmsg_output_counter = 0;			

		ros::AsyncSpinner spinner(4);
		spinner.start();			



	//**************************************************[   ROBOT  ]	

		// *****  JOINT NAMES AND BASE  
		// *****************************************
		const char *setJointNamesA[] = {"left_s0", "left_s1", "left_e0", "left_e1", "left_w0", "left_w1", "left_w2"};
		left__joint_names     = std::vector<std::string>(setJointNamesA, setJointNamesA + 7);	
		const char *setJointNamesB[] = {"right_s0", "right_s1", "right_e0", "right_e1", "right_w0", "right_w1", "right_w2"};
		right_joint_names     = std::vector<std::string>(setJointNamesB, setJointNamesB + 7);	


		// *****  JOINT LIMITS (NOT USED HERE)
		// *****************************************
		CONST_JOINTS_LIMITS_UPPER <<  1.70167993878,  1.047,  3.05417993878,  2.618,  3.059,  2.094, 3.059;
		CONST_JOINTS_LIMITS_LOWER << -1.70167993878, -1.410, -3.05417993878, -0.05, -3.059, -1.57079632679, -3.059;

		// *****  INIT FLAGS
		// *****************************************	
		FLAG_INIT__READING_LEFT_JOINTS  = false; 
		FLAG_INIT__READING_RIGHT_JOINTS = false;
		#ifdef FLAG_COMMUNICATION__REALBAXTER
			FLAG_REAL_BAXTER_COMMUNICATION = true;
			ROS_INFO("[ Baxter-Comm ] REAL BAXTER COMMUNICATION");
		#endif 	
		#ifndef FLAG_COMMUNICATION__REALBAXTER
			FLAG_REAL_BAXTER_COMMUNICATION = false;
			ROS_INFO("[ Baxter-Comm ] GAZEBO COMMUNICATION");
		#endif 	


		// *****  INIT JOINT VALUES
		// *****************************************		
		left_joints  << 0,0,0,0,0,0,0;
		right_joints << 0,0,0,0,0,0,0;
		left_right_joints << 0,0,0,0,0,0,0,  0,0,0,0,0,0,0;
		loopcb_index = 10;


		// *****  JOINT SPEED LIMITS
		// *****************************************					
		double speedcontrollerchange;
		speedcontrollerchange = 90;	
		#ifdef FLAG_COMMUNICATION__REALBAXTER
			speedcontrollerchange = 20;	
			ROS_INFO("[ Baxter-Comm ]:[REAL BAXTER] Setting speed limit to 20");
		#endif 				

		// // Const joint speed limits   (first tryout)
		speedlimits_left[0] = 0.2*speedcontrollerchange;
		speedlimits_left[1] = 0.2*speedcontrollerchange;
		speedlimits_left[2] = 0.2*speedcontrollerchange;	
		speedlimits_left[3] = 0.2*speedcontrollerchange;
		speedlimits_left[4] = 0.3*speedcontrollerchange;	
		speedlimits_left[5] = 0.3*speedcontrollerchange;
		speedlimits_left[6] = 0.5*speedcontrollerchange;
		// for ()
		speedlimits_right[0] = 0.2*speedcontrollerchange;
		speedlimits_right[1] = 0.2*speedcontrollerchange;
		speedlimits_right[2] = 0.2*speedcontrollerchange;	
		speedlimits_right[3] = 0.2*speedcontrollerchange;
		speedlimits_right[4] = 0.3*speedcontrollerchange;	
		speedlimits_right[5] = 0.3*speedcontrollerchange;
		speedlimits_right[6] = 0.5*speedcontrollerchange;		
		//  = speedlimits_left;
}













/*#########################################################
####[ CLASS: baxter_communication ]::[ TYPE: public function  ]
####========================================================
#### PUBLISHES NEW JOINT POSITION IN CORRESPONDING ARM TOPIC
###########################################################*/
void baxter_comm::publish_left_joints(const Eigen::Matrix<double,7,1>& outputJoints)
{
	publish_to_arm(outputJoints, true);
}
void baxter_comm::publish_right_joints(const Eigen::Matrix<double,7,1>& outputJoints)
{
	publish_to_arm(outputJoints, false);
}


void baxter_comm::publish_to_arm(const Eigen::Matrix<double,7,1>& outputJoints, bool left_arm)
{
	int looopint;	

	//*** Adjust output with joint limits
	if (left_arm) {
		for (looopint=0; looopint < 7;looopint++) {
			rosmsg_trajectPoints.positions[looopint] = dqlimspeed(outputJoints(looopint,0),left_joints(looopint,0),(speedlimits_left[looopint]*(MATH_PI/180)));
		}
	}
	else {
		for (looopint=0; looopint < 7;looopint++) {
			rosmsg_trajectPoints.positions[looopint] = dqlimspeed(outputJoints(looopint,0),right_joints(looopint,0),(speedlimits_right[looopint]*(MATH_PI/180)));
		}
	}


	//*********** ADJUST MESSAGE TYPE
	#ifdef FLAG_COMMUNICATION__REALBAXTER
		if (left_arm)
			rosmsg_output.names = left__joint_names;	
		else
			rosmsg_output.names = right_joint_names;	

		rosmsg_output.mode = 1; // 1=pos, 2=vel, 3=torq, 4=raw_pos
		rosmsg_output.command = rosmsg_trajectPoints.positions;	
	#endif 
    #ifndef FLAG_COMMUNICATION__REALBAXTER
		if (left_arm)
			rosmsg_output.joint_names = left__joint_names;
		else
			rosmsg_output.joint_names = right_joint_names;
		
		rosmsg_output.header.stamp = ros::Time::now();
		rosmsg_output.header.seq = rosmsg_output_counter++;
		rosmsg_trajectPoints.time_from_start = ros::Duration(0.3);
		rosmsg_output.points[0] = rosmsg_trajectPoints;
	#endif 

	//*********** PUBLISH MESSAGE
		if (left_arm)
			rospub_output_leftarm_joints.publish(rosmsg_output);		
		else
			rospub_output_rightarm_joints.publish(rosmsg_output);		
}




/*#########################################################
####[ CLASS: baxter_communication ]::[ TYPE: public function  ]
####========================================================
#### PUBLISHES NEW JOINT POSITION IN CORRESPONDING ARM TOPIC
###########################################################*/
void baxter_comm::publish_leftright_joints(const Eigen::Matrix<double,14,1>& outputJoints)
{
	
}







/*#########################################################
####[ CLASS: baxter_communication ]::[ TYPE: public function  ]
####========================================================
#### CALL BACK FUNCTION FOR GATHER JOINTS FROM BAXTER
###########################################################*/
void baxter_comm::callback_getJoints(const sensor_msgs::JointState& sensordata)
{


	//=======================================
	//*******    Get joint name sequence: LEFT ARM
	//=======================================
	for (loopcb=0; loopcb < left__joint_names.size();   loopcb++) {
		loopcb_index = std::find(sensordata.name.begin(), sensordata.name.end(), left__joint_names[loopcb]) - sensordata.name.begin();		
		if( loopcb_index < sensordata.name.size() )
		{	// First check if data is inside boundaries THEN test if you can compare it to the real value
		    if (left__joint_names[loopcb].compare(  sensordata.name[ loopcb_index ]  ) == 0)
		    {		
		    	left__joint_names_Seq7[loopcb] = loopcb_index;	
		    	FLAG_INIT__READING_LEFT_JOINTS = true;

		    	// For the case of multiple inputs in jointstate topic
		    	if ( sensordata.position.size() >= loopcb_index )
		    		left_joints(loopcb,0) = sensordata.position[ left__joint_names_Seq7[loopcb] ];
		    	else 
		    		std::cout  << std::endl  << "[Warning]:[baxter_comm]: Inconsistent number of joint angles (received " << sensordata.position.size() << " - Expected: " << left__joint_names.size() << "). Joint angles (LEFT arm) will not be updated." << std::endl ;
		    }			    
			else {			
			    std::cout  << std::endl  << "[Warning]:[baxter_comm]: Inconsistent joint name. Name will be ignored, and LEFT joint data gathering will not be initialized." << std::endl ;
			    FLAG_INIT__READING_LEFT_JOINTS = false;
			    break;
			}
		}
		else
		{
			FLAG_INIT__READING_LEFT_JOINTS = false;
			break;
		}		
	}
	//=======================================
	//*******    Get joint name sequence: RIGHT ARM
	//=======================================
	for (loopcb=0; loopcb < right_joint_names.size();   loopcb++) {
		loopcb_index = std::find(sensordata.name.begin(), sensordata.name.end(), right_joint_names[loopcb]) - sensordata.name.begin();		
		if( loopcb_index < sensordata.name.size() )
		{	// First check if data is inside boundaries THEN test if you can compare it to the real value
		    if (right_joint_names[loopcb].compare(  sensordata.name[ loopcb_index ]  ) == 0)
		    {		
		    	right_joint_names_Seq7[loopcb] = loopcb_index;	
		    	if (FLAG_INIT__READING_RIGHT_JOINTS==false)
					std::cout<<"teste"<<std::endl;		    		
		    	FLAG_INIT__READING_RIGHT_JOINTS = true;
		    	// For the case of multiple inputs in jointstate topic
		    	if ( sensordata.position.size() >= loopcb_index )
		    		right_joints(loopcb,0) = sensordata.position[ right_joint_names_Seq7[loopcb] ];
		    	else 
		    		std::cout  << std::endl  << "[Warning]:[baxter_comm]: Inconsistent number of joint angles (received " << sensordata.position.size() << " - Expected: " << right_joint_names.size() << "). Joint angles (RIGHT arm) will not be updated." << std::endl ;
		    }			    
			else {			
			    std::cout  << std::endl  << "[Warning]:[baxter_comm]: Inconsistent joint name. Name will be ignored, and RIGHT joint data gathering will not be initialized." << std::endl ;
			    FLAG_INIT__READING_RIGHT_JOINTS = false;
			    break;
			}
		}
		else
		{
			FLAG_INIT__READING_RIGHT_JOINTS = false;
			break;
		}		
	}

}










/*#########################################################
####[ CLASS: baxter_comm ]::[ TYPE: public function  ]
####========================================================
####             INLINE FUNCTIONS
###########################################################	*/
inline float baxter_comm::dqsgn(const float x) 
{
	if (x > 0) return 1.0;
	if (x < 0) return -1.0;
	return 0.0;
}
inline float baxter_comm::dqlimspeed(const float xnew, const float x0, const float maxspeed)
{
	float temp;
	temp = xnew-x0;	
	temp = dqsgn(temp)*fmin(maxspeed,fabs(temp)) + x0;
	return temp;
}



//}; // END OF CLASS





