/**
COOPERATIVE DUAL TASK SPACE

\author Luis Figueredo 
\since 08/2014

*** TODO:
************************
	DQ xr_to_xrbaxe(DQ xr, DQ x2);        
	DQ xrbase_to_xr(DQ xrbase, DQ x2);
************************


*/
#ifndef DQ_CDTS_H
#define DQ_CDTS_H

#include "DQ.h"
#include "DQ_kinematics.h"
#include "DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>




namespace DQ_robotics
{



class DQ_CDTS
{
//************************************ ATTRIBUTES
public: 
	DQ_kinematics   	KINE_ARM_ROBOT_1;
	DQ_kinematics   	KINE_ARM_ROBOT_2;
private: 
	MatrixXd  robot_1__joints;
	MatrixXd  robot_2__joints;
	int   robot_1__num_of_joints;
	int   robot_2__num_of_joints;
	int   two_arms__num_of_joints;


//************************************ METHODS
public:    
	// Class constructors:
	//-------------------------------------------
	DQ_CDTS(){};
    DQ_CDTS(DQ_kinematics ROBOT_1, DQ_kinematics ROBOT_2);
    ~DQ_CDTS(){};

    // Get Pose from individual robots
	//-------------------------------------------
    DQ x1(const Eigen::MatrixXd theta);
    DQ get_pose_x1(const Eigen::MatrixXd theta);
	DQ get_pose_robot1(const Eigen::MatrixXd theta);
	// DQ x1(const Eigen::VectorXd theta);
 //    DQ get_pose_x1(const Eigen::VectorXd theta);    
	// DQ get_pose_robot1(const Eigen::VectorXd theta);

    DQ x2(const Eigen::MatrixXd theta);
    DQ get_pose_x2(const Eigen::MatrixXd theta);
	DQ get_pose_robot2(const Eigen::MatrixXd theta);
	// DQ x2(const Eigen::VectorXd theta);
 //    DQ get_pose_x2(const Eigen::VectorXd theta);    
	// DQ get_pose_robot2(const Eigen::VectorXd theta);


    // Get Pose from Coop dual task space
	//-------------------------------------------
	DQ xr(const Eigen::MatrixXd theta_vec);   
	DQ xr(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2);   
	DQ xa(const Eigen::MatrixXd theta_vec);
	DQ xa(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2);
        
	// Get xr given the base as orientation reference (transform based on the orientation of x2)        
	DQ xr_to_xrbaxe(DQ xr, DQ x2);        
	DQ xrbase_to_xr(DQ xrbase, DQ x2);


    // Get Jacobians
	//-------------------------------------------
	Eigen::MatrixXd  jacobian_1(const Eigen::MatrixXd theta);        
	Eigen::MatrixXd  jacobian_2(const Eigen::MatrixXd theta);     
	Eigen::MatrixXd  jacobian_rel(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2);
	Eigen::MatrixXd  jacobian_rel(const Eigen::MatrixXd theta_vec);        	
	Eigen::MatrixXd  jacobian_abs(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2);
	Eigen::MatrixXd  jacobian_abs(const Eigen::MatrixXd theta_vec);        
       

private:

	Eigen::MatrixXd  check_thetas_1(const Eigen::MatrixXd thetas);
	Eigen::MatrixXd  check_thetas_2(const Eigen::MatrixXd thetas);
	Eigen::MatrixXd  check_thetas(const Eigen::MatrixXd thetas, int expected_size, int init_point, int final_point);

};


}



#endif  
//===> END: DQ_CDTS_H
