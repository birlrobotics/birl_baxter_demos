/**
COOPERATIVE DUAL TASK SPACE

\author Luis Figueredo 
\since 08/2014

*/

#include "DQ_CDTS.h"




namespace DQ_robotics
{


//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : DQ_CDTS               #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  
/// *** CLASS CONSTRUCTOR:  Must receives two DQ_kinematics as input)
DQ_CDTS::DQ_CDTS(DQ_kinematics ROBOT_1, DQ_kinematics ROBOT_2) 
{    
	KINE_ARM_ROBOT_1 = ROBOT_1;
	KINE_ARM_ROBOT_2 = ROBOT_2;

	// Joints:
	robot_1__num_of_joints = KINE_ARM_ROBOT_1.links() - KINE_ARM_ROBOT_1.n_dummy();
	robot_2__num_of_joints = KINE_ARM_ROBOT_2.links() - KINE_ARM_ROBOT_2.n_dummy();
	two_arms__num_of_joints = robot_1__num_of_joints + robot_2__num_of_joints;
	robot_1__joints = MatrixXd(robot_1__num_of_joints,1);
	robot_2__joints = MatrixXd(robot_2__num_of_joints,1);
	// 
	std::cout << "*** Initialization of class DQ_CDTS (Coop dual task space)" << std::endl;
	std::cout << "***      Robot 1 ("<< robot_1__num_of_joints <<" DOFs) AND   Robot 2 ("<<  robot_2__num_of_joints<<" DOFs)" << std::endl;
}



/// ***************************************************************  
/// *****************  
/// *****************  RETURN MatrixXd theta (joint vector) with right size after testing
/// *****************  
/// ***   check_thetas_1(theta): returns the theta with the right size for robot 1 (after testing the size of theta)
/// ***   check_thetas_2(theta): returns the theta with the right size for robot 2 (after testing the size of theta)
/// ***
/// ***************************************************************  
Eigen::MatrixXd DQ_CDTS::check_thetas_1(const Eigen::MatrixXd thetas) {
	return check_thetas(thetas, robot_1__num_of_joints, 0, robot_1__num_of_joints);
}
Eigen::MatrixXd DQ_CDTS::check_thetas_2(const Eigen::MatrixXd thetas) {
	return check_thetas(thetas, robot_2__num_of_joints, robot_1__num_of_joints, robot_2__num_of_joints);
}
///  Real test function
Eigen::MatrixXd DQ_CDTS::check_thetas(const Eigen::MatrixXd thetas, int expected_size, int init_point, int vector_size) {
	MatrixXd return_joints;
	return_joints = MatrixXd::Ones(expected_size,1);  
	return_joints = -1100*return_joints;

	if (thetas.rows() == expected_size)
		return_joints = thetas;
	else {
		if (thetas.rows() == two_arms__num_of_joints)
			return_joints = thetas.block(init_point,0,vector_size,1);	
		else {			
			if (init_point==0)
				std::cout << "****************************************" <<  std::endl << "***[ ERROR ]: Robot 1 - Number of joints ";
			else
				std::cout << "****************************************" <<  std::endl << "***[ ERROR ]: Robot 2 - Number of joints ";
			std::cout << "("<< thetas <<") is different from expected ("<< expected_size <<" or "<< two_arms__num_of_joints <<")" << std::endl;
		}
	}
	return return_joints;
}





/// ***************************************************************  
/// *****************  
/// *****************  RETURN x1 (pose of first arm)
/// *****************  
/// ***   x1(theta): returns the pose of the first arm, where theta is the joint vector
/// ***
/// ***************************************************************  
DQ DQ_CDTS::x1(const Eigen::MatrixXd theta) {
	// robot_1__joints = MatrixXd::Zero(robot_1__num_of_joints,1);    
	// if (theta.rows() == robot_1__num_of_joints)
	// 	robot_1__joints = theta;
	// else {
	// 	if (theta.rows() == two_arms__num_of_joints)
	// 		robot_1__joints = theta.block(0,0,robot_1__num_of_joints,0);	
	// 	else
	// 	{
	// 		std::cout << "****************************************" <<  std::endl;
	// 		std::cout << "***[ ERROR ]: Robot 1 - Number of joints ("<< theta <<") is different from expected ("<< robot_1__num_of_joints <<" or "<< two_arms__num_of_joints <<")" << std::endl;
	// 		return DQ(0);
	// 	}
	// }
	robot_1__joints = check_thetas_1(theta);
	if (robot_1__joints(0,0) < -1000)
		return DQ(0);
	else
		return KINE_ARM_ROBOT_1.fkm(robot_1__joints);
}
DQ DQ_CDTS::get_pose_x1(const Eigen::MatrixXd theta) {
	return x1(theta);
}
DQ DQ_CDTS::get_pose_robot1(const Eigen::MatrixXd theta) {
	return x1(theta);
}




/// ***************************************************************  
/// *****************  
/// *****************  RETURN x2 (pose of second arm)
/// *****************  
/// ***   x2(theta): returns the pose of the second arm, where theta is the joint vector
/// ***
/// ***************************************************************  
DQ DQ_CDTS::x2(const Eigen::MatrixXd theta) {
	robot_2__joints = check_thetas_2(theta);
	if (robot_2__joints(0,0) < -1000)
		return DQ(0);
	else
		return KINE_ARM_ROBOT_2.fkm(robot_2__joints);
}
DQ DQ_CDTS::get_pose_x2(const Eigen::MatrixXd theta) {
	return x2(theta);
}
DQ DQ_CDTS::get_pose_robot2(const Eigen::MatrixXd theta) {
	return x2(theta);
}



/// ***************************************************************  
/// *****************  
/// *****************  RETURN x_relative (relative dual pose between arms)
/// *****************
/// ***
/// ***   * Returns the relative pose using the base of the second arm as base
/// ***   xr(theta):   	     arg(theta) is a MatrixXd with size of combined DOF
/// ***   xr(theta1,theta2): arg(theta1,theta2) are MatrixXd with size of of DOF from robot 1 and 2
/// ***
/// ***************************************************************  
DQ DQ_CDTS::xr(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2)   
{
	DQ pose1, pose2;
	pose1 = x1(theta1);
	pose2 = x2(theta2);
	return pose2.conj()*pose1;	           
}
DQ DQ_CDTS::xr(const Eigen::MatrixXd theta_vec)
{
	robot_1__joints = check_thetas_1(theta_vec);
	robot_2__joints = check_thetas_2(theta_vec);
	return xr(robot_1__joints, robot_2__joints);
}

/// ***************************************************************  
/// *****************  
/// *****************  RETURN x_absolute (absolute dual pose between arms)
/// *****************
/// ***
/// ***   * Returns the absolute pose using the base of the second arm as base
/// ***   xa(theta):   	     arg(theta) is a MatrixXd with size of combined DOF
/// ***   xa(theta1,theta2): arg(theta1,theta2) are MatrixXd with size of of DOF from robot 1 and 2
/// ***
/// ***************************************************************  
DQ DQ_CDTS::xa(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2)   
{
	DQ pose_rel, pose2;
	DQ pose_abs;
	pose_rel = xr(theta1,theta2);
	pose2 = x2(theta2);
	pose_abs = pose2*( pose_rel^(0.5) );	           
	pose_abs = pose_abs*( pose_abs.norm().inv() );
	// std::cout << "disp: " << pose_abs.norm() << std::endl;
	return pose_abs;
}
DQ DQ_CDTS::xa(const Eigen::MatrixXd theta_vec)
{
	robot_1__joints = check_thetas_1(theta_vec);
	robot_2__joints = check_thetas_2(theta_vec);
	return xa(robot_1__joints, robot_2__joints);
}



// /// ***************************************************************  
// /// *****************  
// /// *****************  RETURN xr_to_xrbaxe (relative dual pose given the robot base orientation)
// /// *****************
// /// ***
// /// ***   * Transform xr (rel pose) to xrbase (rel. pose with respect to orientation from the robot base)
// /// ***   xr_to_xrbaxe(DQ xr, DQ x2):    xr is the relative dual pose (with x2 as base), and x2 (robot 2)
// /// ***
// /// ***************************************************************  
// DQ DQ_CDTS::xr_to_xrbaxe(DQ xr, DQ x2)
// {
// 	DQ dqpos = DQ(x2.q(0), x2.q(1), x2.q(2), x2.q(3),  0, 0, 0, 0)*xr;
// 	return xr.P() + E_*0.5*dqpos.translation()*( xr.P() );	
// }     
// DQ DQ_CDTS::xrbase_to_xr(DQ xrbase, DQ x2)
// {

// 	return DQ(1);
// }
// // publishinformation( DQ(dqvar_dq_refxm.q(0), dqvar_dq_refxm.q(1), dqvar_dq_refxm.q(2), dqvar_dq_refxm.q(3),  0, 0, 0, 0)*TWO_ARM_KINE.xr(baxter_ros->left_joints,  baxter_ros->right_joints), true);








/// ***************************************************************  
/// *****************  
/// *****************  RETURN jacobian_X (Jacobian from manipulator x)
/// *****************
/// ***
/// ***   jacobian_X(theta):  arg(theta) is a MatrixXd with size of indivdual manip X or the combined DOF
/// ***
/// ***************************************************************  
Eigen::MatrixXd  DQ_CDTS::jacobian_1(const Eigen::MatrixXd theta)
{
	robot_1__joints = check_thetas_2(theta);
	if (robot_1__joints(0,0) < -1000)
		return MatrixXd::Zero(8,robot_1__num_of_joints);
	else
		return KINE_ARM_ROBOT_1.jacobian(robot_1__joints);
}       
Eigen::MatrixXd  DQ_CDTS::jacobian_2(const Eigen::MatrixXd theta)
{
	robot_2__joints = check_thetas_2(theta);
	if (robot_2__joints(0,0) < -1000)
		return MatrixXd::Zero(8,robot_2__num_of_joints);
	else
		return KINE_ARM_ROBOT_2.jacobian(robot_2__joints);	
}           








/// ***************************************************************  
/// *****************  
/// *****************  RETURN jacobian_rel (Jacobian from Relative Pose)
/// *****************
/// ***
/// ***   jacobian_rel(theta):   	    arg(theta) is a MatrixXd with size of combined DOF
/// ***   jacobian_rel(theta1,theta2):  arg(theta1,theta2) are MatrixXd with size of of DOF from robot 1 and 2
/// ***
/// ***************************************************************  
Eigen::MatrixXd  DQ_CDTS::jacobian_rel(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2)
{
	DQ pose1, pose2;

	Eigen::MatrixXd jacob_part1, jacob_part2, jacob_xr;
	jacob_part1 = MatrixXd(8,robot_1__num_of_joints);
	jacob_part2 = MatrixXd(8,robot_2__num_of_joints);
	jacob_xr 	= MatrixXd::Zero(8,two_arms__num_of_joints);
	pose1 = x1(theta1);
	pose2 = x2(theta2);
	jacob_part1 = Hplus8(pose2.conj())*jacobian_1(theta1);
	jacob_part2 = Hminus8(pose1)*C8()*jacobian_2(theta2);

	jacob_xr.block(0,0,                     8, robot_1__num_of_joints) 	= jacob_part1;
	jacob_xr.block(0,robot_1__num_of_joints,8, robot_1__num_of_joints) 	= jacob_part2;

	return jacob_xr;
}
Eigen::MatrixXd  DQ_CDTS::jacobian_rel(const Eigen::MatrixXd theta_vec)
{
	robot_1__joints = check_thetas_1(theta_vec);
	robot_2__joints = check_thetas_2(theta_vec);
	return jacobian_rel(robot_1__joints, robot_2__joints);
}




/// ***************************************************************  
/// *****************  
/// *****************  RETURN jacobian_abs (Jacobian from Absolute Pose)
/// *****************
/// ***
/// ***   jacobian_abs(theta):   	    arg(theta) is a MatrixXd with size of combined DOF
/// ***   jacobian_abs(theta1,theta2):  arg(theta1,theta2) are MatrixXd with size of of DOF from robot 1 and 2
/// ***
/// ***************************************************************  
Eigen::MatrixXd  DQ_CDTS::jacobian_abs(const Eigen::MatrixXd theta1, const Eigen::MatrixXd theta2)
{
	DQ pose1, pose2, pose_xr;

	Eigen::MatrixXd jacob_part1, jacob_part2;
	Eigen::MatrixXd jacob_xr, jacob_part1a, jacob_part1b, jacob_xr_meio; 
	Eigen::MatrixXd jacob_xa;
	jacob_xr  = MatrixXd(8,two_arms__num_of_joints);
	jacob_part1a = MatrixXd(4,two_arms__num_of_joints);
	jacob_part1b = MatrixXd(4,two_arms__num_of_joints);
	jacob_xr_meio  = MatrixXd::Zero(8,two_arms__num_of_joints);
	jacob_xa  = MatrixXd::Zero(8,two_arms__num_of_joints);

	pose1 = x1(theta1);
	pose2 = x2(theta2);
	pose_xr = pose2.conj()*pose1;
	jacob_xr = jacobian_rel(theta1, theta2);
	//
	// Jacob Xr-meio
	jacob_part1a= 0.5*Hminus4(  pose_xr.P().conj()*( (pose_xr.P())^0.5 )  )*jacob_xr.block(0,0,4,two_arms__num_of_joints);
	jacob_part1b= 0.25*(   Hminus4(  (pose_xr.P())^0.5 )*jacobp(jacob_xr, pose_xr.vec8()) + Hplus4( pose_xr.translation() )*jacob_part1a    );

	jacob_xr_meio.block(0,0,4,two_arms__num_of_joints) 	= jacob_part1a;
	jacob_xr_meio.block(4,0,4,two_arms__num_of_joints) 	= jacob_part1b;
	jacob_xa.block(0,robot_1__num_of_joints,8,robot_2__num_of_joints)   =  Hminus8(pose_xr^(0.5))*jacobian_2(theta2);;
	jacob_xa = jacob_xa + Hplus8(pose2.conj())*jacob_xr_meio;

	return jacob_xa;
}
Eigen::MatrixXd  DQ_CDTS::jacobian_abs(const Eigen::MatrixXd theta_vec)
{
	robot_1__joints = check_thetas_1(theta_vec);
	robot_2__joints = check_thetas_2(theta_vec);
	return jacobian_abs(robot_1__joints, robot_2__joints);
}







} // END OF NAMESPACE


