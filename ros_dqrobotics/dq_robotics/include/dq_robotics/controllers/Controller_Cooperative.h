#ifndef CONTROLLER_COOPERATIVE_H
#define CONTROLLER_COOPERATIVE_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;



namespace DQ_robotics
{


// class Two_Arms
// {

    
// };



class Controller_Cooperative: public DQ_controller
{

//**************************[ Param ]***********
public: 
    MatrixXd jacobian;
    MatrixXd jacobian_INV;
    VectorXd error;

private: 

    //** Control Parameters
    MatrixXd    var_ctrl_Kp,     var_ctrl_Ki,     var_ctrl_Kd;  
    double      var_ctrlgain_kp, var_ctrlgain_ki, var_ctrlgain_kd;
    double      var_ctrl_Ki_memorySize;
    double      var_ctrl_sri_lambda,   var_ctrl_srivar_lambda_max,  var_ctrl_srivar_ballsize;

    //** Task Variables     
    VectorXd error_integral;
    VectorXd error_last_error;
    VectorXd output_joint_speed;
    VectorXd output_joints;
    int  cooperative_number_of_joints;



    //** SUBCLASS: JACOBIAN dqjacob
    //----------------------------------------------
    class dq_indiv_task {
    public:
        bool  enable;
        int size_task, size_num_joints;
        MatrixXd    jacob;
        VectorXd    error;    
        double      kp, ki, kd;
    private:
    public:
        dq_indiv_task(){};
        dq_indiv_task(int num_tasksize, int num_joints) {   
            enable          = false;
            size_task        = num_tasksize;
            size_num_joints  = num_joints;
            error  = VectorXd::Zero(size_task, 1);            
            jacob  = MatrixXd::Zero(size_task, size_num_joints);   
        };
        ~dq_indiv_task(){};      
    private:        
    };
    

    dq_indiv_task task_abs_pose;
    dq_indiv_task task_abs_translation;
    dq_indiv_task task_abs_orientation;
    dq_indiv_task task_rel_pose;
    dq_indiv_task task_rel_translation;
    dq_indiv_task task_rel_orientation;
    dq_indiv_task task_rel_distance;
    dq_indiv_task task_abs_distance;
    //*******************************************************************

    // //** DEBUG MODE
    bool var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF;
    // 
    // //** Constant variables
    // MatrixXd var_const_Identity_4x4;
    // MatrixXd var_const_Identity_8x8;

    //** ROBOTIC ARM Parameters
    // DQ_kinematics   var_ROBOT_KINE;
    // int             var_ROBOT_DOFs;
    //**
    VectorXd        var_ROBOT_JOINTS_LIM__UPPER;
    VectorXd        var_ROBOT_JOINTS_LIM__LOWER;
    VectorXd        var_ROBOT_DUMMY_JOINTS;

    //** Task Variables 
    int var_TASK_SIZE;
    VectorXd    var_task_thetas;
    VectorXd    var_task_thetas_Delta;
    VectorXd var_task_ERROR;
    VectorXd var_task_ki_error;
    VectorXd var_task_kd_last_error;   
    //

    //** FLAGS    
    bool var_FLAG_CTRL__at_least_one_error;
    bool var_FLAG_CTRL__at_least_one_reference;    
    bool var_FLAG_TRACK_enable_tracking_term;
    bool var_FLAG_ROBOT__joint_limits;    

    //** Tracking Terms
    VectorXd var_tracking_updateTerm;
    VectorXd var_tracking4_updateTerm;
    DQ var_tracking_lastReference;    
    DQ var_tracking4_lastReference;


    // //** SUBCLASS: JACOBIAN dqjacob
    // //----------------------------------------------
    // class dqjacob
    // {
    // public:
    //     int task_size, manipDOF;
    //     // Jacob Data
    //     MatrixXd    jacob;
    //     MatrixXd    jacobInv;        
    //     MatrixXd    jacobNSproject; 
    //     // MatrixXd    IDENT_DOFsize; 
    //     // Svd Data
    //     JacobiSVD<MatrixXd> svd;
    //     VectorXd  singValues;
    //     VectorXd  min_U_svd; 
    //     double    min_singValue;        
    // private:
    // public:
    //     dqjacob(){};
    //     dqjacob(int number_DOFs, int input_task_size){ 
    //         task_size = input_task_size;
    //         manipDOF  = number_DOFs;
    //         svd = JacobiSVD<MatrixXd>(number_DOFs, task_size);
    //         jacobNSproject = MatrixXd(number_DOFs, number_DOFs);
    //         jacobInv       = MatrixXd(number_DOFs, task_size);            
    //         jacob          = MatrixXd(task_size,   number_DOFs);   
    //         // IDENT_DOFsize  = Controller_Cooperative::getIDENTMATRIX_WITH_PROPER_SIZE(number_DOFs);
    //     };
    //     ~dqjacob(){};
    //     //** Calculate SVD
    //     void get_SVD(int relevant_DOFs);
    //     //** Inverse Matrix
    //     static MatrixXd get_INV(MatrixXd jacob_data, double lambda );
    //     void get_INV( double lambda );
    //     void get_INV(double lambda, double srivar_lamda_max, double srivar_lamda_region );        
    //     void get_INV_left( double lambda );
    //     void get_NullSpaceProjector() {
    //         MatrixXd IDENTIDADE;
    //         IDENTIDADE = Controller_Cooperative::getIDENTMATRIX_WITH_PROPER_SIZE(manipDOF); 
    //         jacobNSproject = IDENTIDADE - jacobInv*jacob;
    //     }
    //
    //       
    // private:        
    // };
    // dqjacob varJACOB;
    // dqjacob varJACOB4;
    //
    //
    //
    //
    //
    // //** SUBCLASS: Pseudo Robot Kine for joint limit verification
    // //----------------------------------------------
    // class pseudoRobot 
    // {
    // public:  
    //     // Dummy joints
    //     VectorXd dummy_joints_marker;
    //     // Thetas
    //     VectorXd thetas;
    //     VectorXd thetas_delta;
    //     VectorXd thetas_output;       
    //     // Robot config 
    //     MatrixXd dh_matrix;
    //     DQ_kinematics KINE; 
    // private: 
    // public: //methods
    //     pseudoRobot(){};
    //     ~pseudoRobot(){};
    //     // void setData(DQ_kinematics robot, int robot_dofs, VectorXd task_thetas);
    //     void setData(DQ_kinematics robot, int robot_dofs, VectorXd task_thetas)
    //     {
    //         // Dummy joints
    //         dummy_joints_marker = VectorXd::Zero(robot_dofs);
    //         // Thetas
    //         thetas        = task_thetas;
    //         thetas_delta  = VectorXd::Zero(robot_dofs);
    //         thetas_output = VectorXd::Zero(robot_dofs);        
    //         // Robot config 
    //         dh_matrix = robot.getDHMatrix();
    //         KINE = robot;
    //         KINE.set_base(robot.base()); 
    //         KINE.set_effector(robot.effector());
    //     }
    // private: //methods  
    // };    
    // pseudoRobot varPSEUDO_ROBOT;


    
//**************************[ methods ]***********
public: 

    // Controller_Cooperative(DQ_kinematics robot);
    Controller_Cooperative(int num_of_joints);
    ~Controller_Cooperative(){};

    // PUBLIC CONTROL FUNCTIONS
    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas, double POS_GAIN);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas, double POS_GAIN);

    VectorXd get_output_pos(const VectorXd theta1, const VectorXd theta2, double POS_GAIN);
    VectorXd get_output_vel(const VectorXd theta1, const VectorXd theta2, double POS_GAIN);

    // CONFIGURATION SETTINGS
    void set_joint_limits(VectorXd upper_joint_limits, VectorXd lower_joint_limits);
    void set_jacob_srivar_paramconst(const double& sri_lambda);
    void set_jacob_srivar_paramconst(const double& sri_lambda, const double& sriVar_lambda_max, const double& sriVar_ball_size);
    void enableTrackingTerm(const bool bool_falseortrue);
    // SET CONTROL GAINS
    void set_control_gains(double kp);
    void set_control_gains(double kp, double ki);
    void set_control_gains(double kp, double ki, double kd);
    void set_control_gains(double kp, double ki, const double ki_memory, double kd);


    //********************[ SET TASKS: absolute  ]
    void set_task_abs_pose(bool  enable);
    void set_task_abs_dist(bool  enable);
    void set_task_abs_trans(bool  enable);
    void set_task_abs_orien(bool  enable);
    void set_task_abs_pose(bool  enable,   DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs);
    void set_task_abs_dist(bool  enable,   DQ task_xm,  double task_xd,  MatrixXd  Jacob_abs);
    void set_task_abs_trans(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs);
    void set_task_abs_orien(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs);
    VectorXd get_task_abs_error();

    //********************[ SET TASKS: relative  ]
    void set_task_rel_pose(bool  enable);
    void set_task_rel_dist(bool  enable);
    void set_task_rel_trans(bool  enable);
    void set_task_rel_orien(bool  enable);
    void set_task_rel_pose(bool  enable,   DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel);
    void set_task_rel_dist(bool  enable,   DQ task_xm,  double task_xd,  MatrixXd  Jacob_rel);
    void set_task_rel_trans(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel);
    void set_task_rel_orien(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel);
    VectorXd get_task_rel_error();


private: 
    void constructor_jacob_error();

    // void init_default_parameters(DQ_kinematics robot);    
    //** Verify Joint Limits (if enabled)
    // bool joint_limit_verification_step();

};



}


#endif  
//===> END: CONTROLLER_COOPERATIVE_H




