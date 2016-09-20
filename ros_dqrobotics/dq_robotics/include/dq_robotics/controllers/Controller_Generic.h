#ifndef CONTROLLER_GENERIC_H
#define CONTROLLER_GENERIC_H

#include "../DQ.h"
#include "../DQ_kinematics.h"
#include "../DQ_controller.h"
#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;






namespace DQ_robotics
{








class Controller_Generic: public DQ_controller
{

//**************************[ Param ]***********
public: 
private: 

    //** DEBUG MODE
    bool var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF;

    //** Constant variables
    DQ       var_const_DQ_1;
    MatrixXd var_const_Identity_4x4;
    MatrixXd var_const_Identity_8x8;

    //** ROBOTIC ARM Parameters
    DQ_kinematics   var_ROBOT_KINE;
    int             var_ROBOT_DOFs;
    VectorXd        var_ROBOT_JOINTS_LIM__UPPER;
    VectorXd        var_ROBOT_JOINTS_LIM__LOWER;
    VectorXd        var_ROBOT_DUMMY_JOINTS;

    //** Control Parameters
    MatrixXd    var_ctrl_Kp;
    MatrixXd    var_ctrl_Ki;
    double      var_ctrl_Ki_memorySize;
    MatrixXd    var_ctrl_Kd;
    double      var_ctrl_sri_lambda;
    double      var_ctrl_srivar_lambda_max;
    double      var_ctrl_srivar_ballsize;


    //** Task Variables 
    int var_TASK_SIZE;
    VectorXd    var_task_thetas;
    VectorXd    var_task_thetas_Delta;
    VectorXd var_task_ERROR;
    VectorXd var_task_ki_error;
    VectorXd var_task_kd_last_error;   

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


    //** SUBCLASS: JACOBIAN dqjacob
    //----------------------------------------------
    class dqjacob
    {
    public:
        int task_size, manipDOF;
        // Jacob Data
        MatrixXd    jacob;
        MatrixXd    jacobInv;        
        MatrixXd    jacobNSproject; 
        // MatrixXd    IDENT_DOFsize; 
        // Svd Data
        JacobiSVD<MatrixXd> svd;
        VectorXd  singValues;
        VectorXd  min_U_svd; 
        double    min_singValue;        
    private:
    public:
        dqjacob(){};
        dqjacob(int number_DOFs, int input_task_size){ 
            task_size = input_task_size;
            manipDOF  = number_DOFs;
            svd = JacobiSVD<MatrixXd>(number_DOFs, task_size);
            jacobNSproject = MatrixXd(number_DOFs, number_DOFs);
            jacobInv       = MatrixXd(number_DOFs, task_size);            
            jacob          = MatrixXd(task_size,   number_DOFs);   
            // IDENT_DOFsize  = Controller_Generic::getIDENTMATRIX_WITH_PROPER_SIZE(number_DOFs);
        };
        ~dqjacob(){};
        //** Calculate SVD
        void get_SVD(int relevant_DOFs);
        //** Inverse Matrix
        static MatrixXd get_INV(MatrixXd jacob_data, double lambda );
        void get_INV( double lambda );
        void get_INV(double lambda, double srivar_lamda_max, double srivar_lamda_region );        
        void get_INV_left( double lambda );
        void get_NullSpaceProjector() {
            MatrixXd IDENTIDADE;
            IDENTIDADE = Controller_Generic::getIDENTMATRIX_WITH_PROPER_SIZE(manipDOF); 
            jacobNSproject = IDENTIDADE - jacobInv*jacob;
        }

       
    private:        
    };
    dqjacob varJACOB;
    dqjacob varJACOB4;





    //** SUBCLASS: Pseudo Robot Kine for joint limit verification
    //----------------------------------------------
    class pseudoRobot 
    {
    public:  
        // Dummy joints
        VectorXd dummy_joints_marker;
        // Thetas
        VectorXd thetas;
        VectorXd thetas_delta;
        VectorXd thetas_output;       
        // Robot config 
        MatrixXd dh_matrix;
        DQ_kinematics KINE; 
    private: 
    public: //methods
        pseudoRobot(){};
        ~pseudoRobot(){};
        // void setData(DQ_kinematics robot, int robot_dofs, VectorXd task_thetas);
        void setData(DQ_kinematics robot, int robot_dofs, VectorXd task_thetas)
        {
            // Dummy joints
            dummy_joints_marker = VectorXd::Zero(robot_dofs);
            // Thetas
            thetas        = task_thetas;
            thetas_delta  = VectorXd::Zero(robot_dofs);
            thetas_output = VectorXd::Zero(robot_dofs);        
            // Robot config 
            dh_matrix = robot.getDHMatrix();
            KINE = robot;
            KINE.set_base(robot.base()); 
            KINE.set_effector(robot.effector());
        }
    private: //methods  
    };    
    pseudoRobot varPSEUDO_ROBOT;


    
//**************************[ methods ]***********
public: 


    Controller_Generic(DQ_kinematics robot);
    ~Controller_Generic(){};

    // PUBLIC CONTROL FUNCTIONS
    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointPositions( const DQ reference, const VectorXd thetas, double POS_GAIN);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas);
    VectorXd getNewJointVelocities( const DQ reference, const VectorXd thetas, double POS_GAIN);

    // CONFIGURATION SETTINGS
    void set_joint_limits(VectorXd upper_joint_limits, VectorXd lower_joint_limits);
    void set_jacob_srivar_paramconst(const double& sri_lambda);
    void set_jacob_srivar_paramconst(const double& sri_lambda, const double& sriVar_lambda_max, const double& sriVar_ball_size);
    void enableTrackingTerm(const bool bool_falseortrue);
    // SET CONTROL GAINS
    void set_control_gains(const MatrixXd& kp);
    void set_control_gains(const MatrixXd& kp, const MatrixXd& ki);
    void set_control_gains(const MatrixXd& kp, const MatrixXd& ki, const MatrixXd& kd);    
    void set_control_gains(const MatrixXd& kp, const MatrixXd& ki, const double ki_memory, const MatrixXd& kd);
    void set_control_gains(double kp);
    void set_control_gains(double kp, double ki);
    void set_control_gains(double kp, double ki, double kd);
    void set_control_gains(double kp, double ki, const double ki_memory, double kd);


    static MatrixXd getIDENTMATRIX_WITH_PROPER_SIZE(int MSIZE) 
    {
        MatrixXd IDENTIDADE;
        if (MSIZE<=5)
            if (MSIZE==1)
                IDENTIDADE = Matrix<double,1,1>::Identity(); 
            else 
                if (MSIZE==2)
                    IDENTIDADE = Matrix<double,2,2>::Identity(); 
                else 
                    if (MSIZE==3)
                        IDENTIDADE = Matrix<double,3,3>::Identity(); 
                    else 
                        if (MSIZE==4)
                            IDENTIDADE = Matrix<double,4,4>::Identity(); 
                        else 
                            if (MSIZE==5)
                                IDENTIDADE = Matrix<double,5,5>::Identity();                             
                            else 
                                std::cout << "[ERROR]: getIDENTMATRIX_WITH_PROPER_SIZE => Cannot use negative value for size of matrix (MSIZE)" << std::endl;
        else if (MSIZE > 5 && MSIZE<=10)
            if (MSIZE==6)
                IDENTIDADE = Matrix<double,6,6>::Identity(); 
            else 
                if (MSIZE==7)
                    IDENTIDADE = Matrix<double,7,7>::Identity();
                    else 
                        if (MSIZE==8)
                            IDENTIDADE = Matrix<double,8,8>::Identity();  
                        else 
                            if (MSIZE==9)
                                IDENTIDADE = Matrix<double,9,9>::Identity(); 
                            else 
                                if (MSIZE==10)
                                    IDENTIDADE = Matrix<double,10,10>::Identity();                                    
        else if (MSIZE > 10)       
            if (MSIZE==11)
                IDENTIDADE = Matrix<double,11,11>::Identity();
            else 
                if (MSIZE==12)
                    IDENTIDADE = Matrix<double,12,12>::Identity();  
                else 
                    if (MSIZE==13)
                        IDENTIDADE = Matrix<double,13,13>::Identity(); 
                    else 
                        if (MSIZE==14)
                            IDENTIDADE = Matrix<double,14,14>::Identity();  
                        else 
                            if (MSIZE==15)
                                IDENTIDADE = Matrix<double,15,15>::Identity();  
                            else 
                                if (MSIZE==16)
                                    IDENTIDADE = Matrix<double,16,16>::Identity();  
                                else
                                    std::cout << "[ERROR]: getIDENTMATRIX_WITH_PROPER_SIZE => Unfortunately, function has not been defined for matrix size over 16 (MSIZE>16)" << std::endl;                                
        // IDENT_DOFsize  = Matrix<double,number_DOFs,number_DOFs>::Identity(); 
        return  IDENTIDADE;
    }


private: 

    void init_default_parameters(DQ_kinematics robot);    
    void print_parameters();
    MatrixXd get_IDENT()  {
        if (var_TASK_SIZE==8)
            return var_const_Identity_8x8;
        else
            return var_const_Identity_4x4;
    }

    //** Verify Joint Limits (if enabled)
    bool joint_limit_verification_step();

};



}


#endif  
//===> END: CONTROLLER_GENERIC_H
