#ifndef CONTROLLER_INVARIANT_PID_SRIVAR_TRACKING_H
#define CONTROLLER_INVARIANT_PID_SRIVAR_TRACKING_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;



namespace DQ_robotics
{



class Controller_invariant_Pid_SRIvar_Tracking : public DQ_controller
{

public: //variables

private: //variables

    DQ_kinematics robot_;
    int robot_dofs_;

    MatrixXd kp_;
    MatrixXd ki_;
    double ki_memory_;
    MatrixXd kd_;
    double beta_;
    double lambda_max_;
    double epsilon_;

    //Joint Limit Related
    VectorXd upper_joint_limits_;
    VectorXd lower_joint_limits_;
    VectorXd original_dummy_joints_;

    VectorXd thetas_;
    VectorXd delta_thetas_;

    VectorXd error_;
    VectorXd integral_error_;
    VectorXd last_error_;
    bool at_least_one_error_;
    DQ last_reference_;
    bool at_least_one_reference_;    
    VectorXd trackingTerm_;
    bool enable_trackintTerm;


    MatrixXd task_jacobian_;
    MatrixXd task_jacobian_pseudoinverse_;

    JacobiSVD<MatrixXd> svd_;
    VectorXd singular_values_;
    MatrixXd svd_sigma_inverted_;
    MatrixXd identity_;

    DQ end_effector_pose_;

    DQ dq_one_;

public: //methods

    // Controller_invariant_Pid_SRIvar_Tracking(){};
    Controller_invariant_Pid_SRIvar_Tracking(DQ_kinematics robot, VectorXd upper_joint_limits, VectorXd lower_joint_limits, 
                                                           MatrixXd feedback_gain, double beta, double lambda_max, double epsilon);
    Controller_invariant_Pid_SRIvar_Tracking( const DQ_kinematics& robot, VectorXd upper_joint_limits, VectorXd lower_joint_limits, const MatrixXd& kp, const MatrixXd& ki, const double ki_memory, const MatrixXd& kd, const double& beta, const double& lambda_max, const double& epsilon);

    ~Controller_invariant_Pid_SRIvar_Tracking(){};

    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);

    void enableTrackingTerm(const bool bool_falseortrue);

private: //methods

};



}


#endif  
//===> END: CONTROLLER_INVARIANT_PID_SRIVAR_TRACKING_H
