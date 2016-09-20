#include "Controller_Cooperative.h"
// #include <Eigen/Dense>
// #include <iostream>

using namespace Eigen;




namespace DQ_robotics
{


//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################====================================#########################################
//#################################                                    #########################################
//#################################  CLASS : Controller_Cooperative    #########################################
//#################################                                    #########################################
//#################################====================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  
/// * CALL init_default_parameters with robot input (DQ_kinematics)
//*********************************************************************************************************
// Controller_Cooperative::Controller_Cooperative(DQ_kinematics robot) : DQ_controller()
// {    
//     init_default_parameters(robot);
//     print_parameters();
// }
Controller_Cooperative::Controller_Cooperative(int num_of_joints) : DQ_controller()
{    
    //** NUMBER OF JOINTS (COOPERATIVE)
    cooperative_number_of_joints = num_of_joints;
    //** DEBUG MODE:
    var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF = true;
    //** Init Flags   ****[ DEFAULT VALUES ]
    var_FLAG_CTRL__at_least_one_error     = false;    
    var_FLAG_CTRL__at_least_one_reference = false;
    var_FLAG_TRACK_enable_tracking_term   = false;
    var_FLAG_ROBOT__joint_limits          = false;    

    //** Control Param ****[ DEFAULT VALUES ]
    var_ctrlgain_kp = 0.05;
    var_ctrlgain_ki = 0.0; 
    var_ctrlgain_kd = 0.0;
    var_ctrl_Ki_memorySize = 100.0;
    var_ctrl_sri_lambda       = 0.01;
    var_ctrl_srivar_lambda_max = 0.0; //0.05;
    var_ctrl_srivar_ballsize  = 0.001;

    //** INIT VARIABLES WITHOUT RIGHT SIZE
    jacobian = MatrixXd::Zero(1,num_of_joints);
    jacobian_INV = MatrixXd::Zero(num_of_joints,1);
    error = VectorXd::Zero(1,1);
    error_integral   = VectorXd::Zero(1,1);
    error_last_error = VectorXd::Zero(1,1);
    output_joint_speed  = VectorXd::Zero(num_of_joints,1);
    output_joints       = VectorXd::Zero(num_of_joints,1);
    
    task_abs_pose        = dq_indiv_task(8,num_of_joints);
    task_abs_translation = dq_indiv_task(4,num_of_joints);
    task_abs_orientation = dq_indiv_task(4,num_of_joints);
    task_abs_distance    = dq_indiv_task(1,num_of_joints);
    task_rel_pose          = dq_indiv_task(4,num_of_joints);
    task_rel_translation   = dq_indiv_task(4,num_of_joints);
    task_rel_orientation   = dq_indiv_task(4,num_of_joints);
    task_rel_distance      = dq_indiv_task(1,num_of_joints);


    // INIT JOINT VECTORS
    var_task_thetas         = VectorXd::Zero(num_of_joints,1);
    var_task_thetas_Delta   = VectorXd::Zero(num_of_joints,1);        

    // //** Tracking Terms
    // var_tracking_updateTerm      = MatrixXd::Zero(8,1);    
    // var_tracking_lastReference   = DQ(0,0,0,0,0,0,0,0);    
    // var_tracking4_updateTerm      = MatrixXd::Zero(4,1);    
    // var_tracking4_lastReference   = DQ(0,0,0,0);   

}







                  
//------------------------------------------------------------------------------------------------------------
//#############################################################################################################
//#################################===================================#########################################
//#################################                                   #########################################
//#################################  CLASS : Controller_Cooperative   #########################################
//#################################        SET TASKS : ABSOLUTE       #########################################
//#################################                                   #########################################
//#################################===================================#########################################
//#############################################################################################################
//-------------------------------------------------------------------------------------------------------------
void Controller_Cooperative::set_task_abs_pose(bool  enable) {
    if (enable==false)
        task_abs_pose.enable = false;
}
void Controller_Cooperative::set_task_abs_dist(bool  enable){
    if (enable==false)
        task_abs_distance.enable = false;
}
void Controller_Cooperative::set_task_abs_trans(bool  enable){
    if (enable==false)
        task_abs_translation.enable = false;
}
void Controller_Cooperative::set_task_abs_orien(bool  enable){
    if (enable==false)
        task_abs_orientation.enable = false;    
}
//######################################################################################################
//######################################################################################################
//######################################################################################################
void Controller_Cooperative::set_task_abs_pose(bool  enable,   DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs){
    if (enable==false) {
        task_abs_pose.enable = false;
        return;
    }
    task_abs_pose.enable = true;
    task_abs_pose.jacob = Hminus8(task_xd)*C8()*Jacob_abs;
    task_abs_pose.error = vec8(DQ(1)-task_xm.conj()*task_xd); 
    // std::cout << "test err abs: " << task_abs_pose.error << std::endl; 
    // std::cout << "test pose xm: " << task_xm << std::endl;
    // std::cout << "test pose xd: " << task_xd << std::endl;

}

void Controller_Cooperative::set_task_abs_dist(bool  enable,   DQ task_xm,  double task_xd,  MatrixXd  Jacob_abs){
    if (enable==false) {
        task_abs_distance.enable = false;
        return;
    }
    // task_abs_distance.enable = true;
    // task_abs_distance.jacob = jacobd(Jacob_abs, task_xm.vec8());    
    // task_abs_distance.error = - ( task_xd - pow(task_xm.translation().norm(),2)) ;     
}
void Controller_Cooperative::set_task_abs_trans(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs){  // TASK_XD in this case must be a translation already!
    if (enable==false) {
        task_abs_translation.enable = false;
        return;
    }
    task_abs_translation.enable = true;
    task_abs_translation.jacob = jacobp(Jacob_abs, task_xm.vec8());    
    task_abs_translation.error = vec4(task_xd - task_xm.translation());  
}
void Controller_Cooperative::set_task_abs_orien(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_abs){
    if (enable==false) {
        task_abs_orientation.enable = false;
        return;
    }
    task_abs_orientation.enable = true;
    task_abs_orientation.jacob = Hminus4(task_xd)*C4()*Jacob_abs.block(0,0,4,task_abs_orientation.size_num_joints);
    task_abs_orientation.error = vec4(DQ(1)-task_xm.P().conj()*task_xd.P());      
}


                  
//------------------------------------------------------------------------------------------------------------
//#############################################################################################################
//#################################===================================#########################################
//#################################                                   #########################################
//#################################  CLASS : Controller_Cooperative   #########################################
//#################################        SET TASKS : ABSOLUTE       #########################################
//#################################                                   #########################################
//#################################===================================#########################################
//#############################################################################################################
//-------------------------------------------------------------------------------------------------------------  
    void Controller_Cooperative::set_task_rel_pose(bool  enable) {
    if (enable==false)
        task_rel_pose.enable = false;
}
void Controller_Cooperative::set_task_rel_dist(bool  enable){
    if (enable==false)
        task_rel_distance.enable = false;
}
void Controller_Cooperative::set_task_rel_trans(bool  enable){
    if (enable==false)
        task_rel_translation.enable = false;
}
void Controller_Cooperative::set_task_rel_orien(bool  enable){
    if (enable==false)
        task_rel_orientation.enable = false;    
}
//######################################################################################################
//######################################################################################################
//######################################################################################################
void Controller_Cooperative::set_task_rel_pose(bool  enable,   DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel){
    if (enable==false) {
        task_rel_pose.enable = false;
        return;
    }
    task_rel_pose.enable = true;
    task_rel_pose.jacob = Hminus8(task_xd)*C8()*Jacob_rel;
    task_rel_pose.error = 3*vec8(DQ(1)-task_xm.conj()*task_xd);  
}

void Controller_Cooperative::set_task_rel_dist(bool  enable,   DQ task_xm,  double task_xd,  MatrixXd  Jacob_rel){
    if (enable==false) {
        task_rel_distance.enable = false;
        return;
    }
    // task_rel_distance.enable = true;
    // task_rel_distance.jacob = jacobd(Jacob_rel, task_xm.vec8());    
    // task_rel_distance.error = - ( task_xd - pow(task_xm.translation().norm(),2)) ;     
}
void Controller_Cooperative::set_task_rel_trans(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel){  // TASK_XD in this case must be a translation already!
    if (enable==false) {
        task_rel_translation.enable = false;
        return;
    }
    task_rel_translation.enable = true;
    task_rel_translation.jacob = jacobp(Jacob_rel, task_xm.vec8());    
    task_rel_translation.error = vec4(task_xd - task_xm.translation());  
}
void Controller_Cooperative::set_task_rel_orien(bool  enable,  DQ task_xm,  DQ task_xd,  MatrixXd  Jacob_rel)
{
    if (enable==false) {
        task_rel_orientation.enable = false;
        return;
    }
    task_rel_orientation.enable = true;
    task_rel_orientation.jacob = Hminus4(task_xd)*C4()*Jacob_rel.block(0,0,4,task_rel_orientation.size_num_joints);
    task_rel_orientation.error = vec4(DQ(1)-task_xm.P().conj()*task_xd.P());    
}







//------------------------------------------------------------------------------------------------------------
//#############################################################################################################
//#################################===================================#########################################
//#################################                                   #########################################
//#################################  CLASS : Controller_Cooperative   #########################################
//#################################      GET ERROS                    #########################################
//#################################                                   #########################################
//#################################===================================#########################################
//#############################################################################################################
//-------------------------------------------------------------------------------------------------------------  
VectorXd Controller_Cooperative::get_task_abs_error()
{
    VectorXd abs_error;
    int size_dualtask_abs;
    size_dualtask_abs = 0;    
    // Check task size
    if (task_abs_pose.enable)
        size_dualtask_abs = size_dualtask_abs + 8;
    else {
        if (task_abs_translation.enable)
            size_dualtask_abs = size_dualtask_abs + 4;
        else {
            if (task_abs_distance.enable)
                size_dualtask_abs = size_dualtask_abs + 1;
        }
        if (task_abs_orientation.enable)
            size_dualtask_abs = size_dualtask_abs + 4;
    }    
    abs_error = VectorXd::Zero(size_dualtask_abs, 1);
    int cur_pos=0;
    // *** GET ABSOLUTE ERROR
    if (task_abs_pose.enable)     {
        abs_error.block(cur_pos,0,8,1)   = task_abs_pose.error;
        cur_pos = cur_pos + 8;
    }
    else   {
        if (task_abs_translation.enable)  {
            abs_error.block(cur_pos,0,4,1)   = task_abs_translation.error;
            cur_pos = cur_pos + 4;
        }
        else {
            if (task_abs_distance.enable)  
                std::cout << "Not yet implemented" << std::endl;
        }
        if (task_abs_orientation.enable)  
            abs_error.block(cur_pos,0,4,1)   = task_abs_orientation.error;
    }    
    return abs_error;
}


VectorXd Controller_Cooperative::get_task_rel_error()
{
    VectorXd rel_error;
    int size_dualtask_rel;
    size_dualtask_rel = 0;
    // Check task size
    if (task_rel_pose.enable)
        size_dualtask_rel = size_dualtask_rel + 8;
    else {
        if (task_rel_translation.enable)
            size_dualtask_rel = size_dualtask_rel + 4;
        else {
            if (task_rel_distance.enable)
                size_dualtask_rel = size_dualtask_rel + 1;
        }
        if (task_rel_orientation.enable)
            size_dualtask_rel = size_dualtask_rel + 4;
    }    
    rel_error = VectorXd::Zero(size_dualtask_rel, 1);
    int cur_pos=0;    
    // *** RELATIVE    
    if (task_rel_pose.enable)     {
        rel_error.block(cur_pos,0,8,1)  = task_rel_pose.error;
        cur_pos = cur_pos + 8;
    }
    else   {
        if (task_rel_translation.enable)  {
            rel_error.block(cur_pos,0,4,1) = task_rel_translation.error;
            cur_pos = cur_pos + 4;
        }
        else {
            if (task_rel_distance.enable)  
                std::cout << "Not yet implemented" << std::endl;
        }
        if (task_rel_orientation.enable)  
            rel_error.block(cur_pos,0,4,1)  = task_rel_orientation.error;
    }    
    return rel_error;
}




                  
//------------------------------------------------------------------------------------------------------------
//#############################################################################################################
//#################################===================================#########################################
//#################################                                   #########################################
//#################################  CLASS : Controller_Cooperative   #########################################
//#################################      CONSTRUCTOR JACOB-ERROR      #########################################
//#################################                                   #########################################
//#################################===================================#########################################
//#############################################################################################################
//-------------------------------------------------------------------------------------------------------------  
void Controller_Cooperative::constructor_jacob_error()
{
    int size_dualtask;
    size_dualtask = 0;

    // Check task size
    if (task_abs_pose.enable)
        size_dualtask = size_dualtask + 8;
    else {
        if (task_abs_translation.enable)
            size_dualtask = size_dualtask + 4;
        else {
            if (task_abs_distance.enable)
                size_dualtask = size_dualtask + 1;
        }
        if (task_abs_orientation.enable)
            size_dualtask = size_dualtask + 4;
    }
    if (task_rel_pose.enable)
        size_dualtask = size_dualtask + 8;
    else {
        if (task_rel_translation.enable)
            size_dualtask = size_dualtask + 4;
        else {
            if (task_rel_distance.enable)
                size_dualtask = size_dualtask + 1;
        }
        if (task_rel_orientation.enable)
            size_dualtask = size_dualtask + 4;
    }

    // RESIZE JACOBIAN AND ERROR
    if (error_integral.rows() !=  size_dualtask)
    {
        error_integral.resize(size_dualtask, NoChange);
        error_integral = VectorXd::Zero(size_dualtask,1);
    }
    

    error.resize(size_dualtask, NoChange);
    jacobian.resize(size_dualtask, NoChange);
    jacobian_INV.resize(NoChange, size_dualtask);
    
    int cur_pos=0;
    // CREATE DUAL TASK JACOB AND ERROR
    // *** ABSOLUTE
    if (task_abs_pose.enable)     {
        error.block(cur_pos,0,8,1)                               = task_abs_pose.error;
        jacobian.block(cur_pos,0,8,cooperative_number_of_joints) = task_abs_pose.jacob;
        cur_pos = cur_pos + 8;
    }
    else   {
        if (task_abs_translation.enable)  {
            error.block(cur_pos,0,4,1)                               = task_abs_translation.error;
            jacobian.block(cur_pos,0,4,cooperative_number_of_joints) = task_abs_translation.jacob;
            cur_pos = cur_pos + 4;
        }
        else {
            if (task_abs_distance.enable)  {
                std::cout << "Not yet implemented" << std::endl;
            }
        }
        if (task_abs_orientation.enable)  {
            error.block(cur_pos,0,4,1)                               = task_abs_orientation.error;
            jacobian.block(cur_pos,0,4,cooperative_number_of_joints) = task_abs_orientation.jacob;
            cur_pos = cur_pos + 4;            
        }
    }    
    // *** RELATIVE    
    if (task_rel_pose.enable)     {
        error.block(cur_pos,0,8,1)                               = task_rel_pose.error;
        jacobian.block(cur_pos,0,8,cooperative_number_of_joints) = task_rel_pose.jacob;
        cur_pos = cur_pos + 8;
    }
    else   {
        if (task_rel_translation.enable)  {
            error.block(cur_pos,0,4,1)                               = task_rel_translation.error;
            jacobian.block(cur_pos,0,4,cooperative_number_of_joints) = task_rel_translation.jacob;
            cur_pos = cur_pos + 4;
        }
        else {
            if (task_rel_distance.enable)  {
                std::cout << "Not yet implemented" << std::endl;
            }
        }
        if (task_rel_orientation.enable)  {
            error.block(cur_pos,0,4,1)                               = task_rel_orientation.error;
            jacobian.block(cur_pos,0,4,cooperative_number_of_joints) = task_rel_orientation.jacob;
            cur_pos = cur_pos + 4;            
        }
    }    
}
















// /**********************************************************************************
// ####################################################################################
// ####  CLASS:   Controller_Cooperative                 
// ####  FUNCTION:   init_default_parameters(DQ_kinematics robot) 
// ####_______________________________________________________________________________  
//     * SET DEFAULT VALUES FOR THE PARAMETERS OF THE CLASS
// ####################################################################################    
// ***********************************************************************************/
// void Controller_Cooperative::init_default_parameters(DQ_kinematics robot) 
// {    
//     //** DEBUG MODE:
//     var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF = true;

//     var_TASK_SIZE = 8;

//     //** Init Flags   ****[ DEFAULT VALUES ]
//     var_FLAG_CTRL__at_least_one_error     = false;    
//     var_FLAG_CTRL__at_least_one_reference = false;
//     var_FLAG_TRACK_enable_tracking_term   = false;
//     var_FLAG_ROBOT__joint_limits          = false;


//     //** Init Robotic Arm Param
//     var_ROBOT_DOFs     = (robot.links() - robot.n_dummy());
//     var_ROBOT_KINE     = robot;
//     var_ROBOT_JOINTS_LIM__UPPER    = VectorXd::Zero(var_ROBOT_DOFs); // upper_joint_limits;
//     var_ROBOT_JOINTS_LIM__LOWER    = VectorXd::Zero(var_ROBOT_DOFs); // lower_joint_limits;
//     var_ROBOT_DUMMY_JOINTS         = var_ROBOT_KINE.dummy(); 

//     //** Control Param ****[ DEFAULT VALUES ]
//     var_ctrl_Kp     = MatrixXd::Identity(8,8);
//     var_ctrl_Kp     = 0.1*var_ctrl_Kp;
//     var_ctrl_Ki     = MatrixXd::Zero(8,8);
//     var_ctrl_Kd     = MatrixXd::Zero(8,8);
//     var_ctrl_Ki_memorySize = 100.0;
//     var_ctrl_sri_lambda       = 0.01;
//     var_ctrl_srivar_lambda_max = 0.0; //0.05;
//     var_ctrl_srivar_ballsize  = 0.001;

//     //** Init evolution Task variables
//     var_task_thetas         = MatrixXd(var_ROBOT_DOFs,1);
//     var_task_thetas_Delta   = MatrixXd::Zero(var_ROBOT_DOFs,1);        
//     var_task_ERROR          = MatrixXd(8,1);
//     var_task_ki_error       = MatrixXd::Zero(8,1);
//     var_task_kd_last_error  = MatrixXd::Zero(8,1);

//     // // TASK JACOB 
//     varJACOB  = dqjacob(var_ROBOT_DOFs, 8);
//     varJACOB4 = dqjacob(var_ROBOT_DOFs, 4);


//     //** Tracking Terms
//     var_tracking_updateTerm      = MatrixXd::Zero(8,1);    
//     var_tracking_lastReference   = DQ(0,0,0,0,0,0,0,0);    
//     var_tracking4_updateTerm      = MatrixXd::Zero(4,1);    
//     var_tracking4_lastReference   = DQ(0,0,0,0);   
// }







                  
//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : Controller_Cooperative    #########################################
//#################################         CONFIGURATION          #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  


/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Cooperative                 
####  FUNCTION:   set_joint_limits (VectorXd:  upper_limits and lower_limits )
####_______________________________________________________________________________  
    * SET the arm joint limits and enable joint limits exclusion
***********************************************************************************/
void Controller_Cooperative::set_joint_limits(VectorXd upper_joint_limits, VectorXd lower_joint_limits)
{
    var_FLAG_ROBOT__joint_limits  = true;
    var_ROBOT_JOINTS_LIM__UPPER   = upper_joint_limits;
    var_ROBOT_JOINTS_LIM__LOWER   = lower_joint_limits;
}



/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Cooperative                 
####  FUNCTION:   set_control_gains (different options)
####_______________________________________________________________________________  
    * Adjust control gains of the PID (just P matrix)
***********************************************************************************/
/// * Adjust control gains of the PID (just P gain-scalar)
void Controller_Cooperative::set_control_gains(double kp){
    var_ctrlgain_kp             = kp;
}
/// * Adjust control gains of the PID (just Kp and Ki gain-scalars)
void Controller_Cooperative::set_control_gains(double kp, double ki){
    var_ctrlgain_kp             = kp;
    var_ctrlgain_ki             = ki;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd gain-scalars)
void Controller_Cooperative::set_control_gains(double kp, double ki, double kd){
    var_ctrlgain_kp             = kp;
    var_ctrlgain_ki             = ki;
    var_ctrlgain_kd             = kd;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd gain-scalars | and  integral memory it)
void Controller_Cooperative::set_control_gains(double kp, double ki, const double ki_memory, double kd){
    set_control_gains(kp, ki, kd);
    var_ctrl_Ki_memorySize  = ki_memory;   
}






/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Cooperative                 
####  FUNCTION:   set_jacob_srivar_paramconst (double:  sri_lambda, srivar_lambda_max, sri_var_lambda_region)
####_______________________________________________________________________________  
    * Adjust damping parameters for the SRI variable inverse  
####################################################################################    
***********************************************************************************/
void Controller_Cooperative::set_jacob_srivar_paramconst(const double& sri_lambda, const double& sriVar_lambda_max, const double& sriVar_ball_size)
{
    var_ctrl_sri_lambda          = sri_lambda;
    var_ctrl_srivar_lambda_max   = sriVar_lambda_max;
    var_ctrl_srivar_ballsize     = sriVar_ball_size;
}
/// * Adjust damping parameter for the SRI  inverse  
void Controller_Cooperative::set_jacob_srivar_paramconst(const double& sri_lambda)
{
    var_ctrl_sri_lambda          = sri_lambda;
}
//***************************************************************************************************************








//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : Controller_Cooperative    #########################################
//#################################         CONTROLLERS            #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------
VectorXd Controller_Cooperative::getNewJointPositions(const DQ reference, const VectorXd thetas)
{
    var_task_thetas_Delta = getNewJointVelocities(reference, thetas, 1.0);
    // Send updated thetas to simulation
    return (var_task_thetas + var_task_thetas_Delta);
}

VectorXd Controller_Cooperative::getNewJointPositions(const DQ reference, const VectorXd thetas, double POS_GAIN)
{
    var_task_thetas_Delta = getNewJointVelocities(reference, thetas, POS_GAIN);
    // Send updated thetas to simulation
    return (var_task_thetas + var_task_thetas_Delta);
}
VectorXd Controller_Cooperative::getNewJointVelocities(const DQ reference, const VectorXd thetas)
{
    return getNewJointVelocities( reference, thetas, 1.0);     
}


VectorXd Controller_Cooperative::getNewJointVelocities(const DQ reference, const VectorXd thetas, double POS_GAIN)
{
    var_task_thetas = thetas; 
    return var_task_thetas;
}




VectorXd Controller_Cooperative::get_output_pos(const VectorXd theta1, const VectorXd theta2, double POS_GAIN)
{
    var_task_thetas_Delta = get_output_vel(theta1, theta2, POS_GAIN);
    // Send updated thetas to simulation
    return (var_task_thetas + var_task_thetas_Delta);
}
VectorXd Controller_Cooperative::get_output_vel(const VectorXd theta1, const VectorXd theta2, double POS_GAIN)
{
    // Init control step:
    // var_task_thetas = thetas; //This is necessary for the getNewJointPositions to work
    var_task_thetas.block(0,            0,theta1.rows(),1) = theta1;
    var_task_thetas.block(theta1.rows(),0,theta2.rows(),1) = theta2;
    var_task_thetas_Delta = VectorXd::Zero(14,1);
    constructor_jacob_error();

    // std::cout << "teste:  size of jacob: " << jacobian.rows() << "x" << jacobian.cols() << std::endl;
    // std::cout << "teste:  error: " << error(0,0) << std::endl;

    // NULL SPACE VARIABLES 
    //*************************************************
    // MatrixXd NS_Z2;     //  7 x 1
    // MatrixXd NS_OPT_FCT; // n x 1
    // MatrixXd NS_JACOB;  //  n x 7
    // VectorXd        NS_THETA_MEDIO;
    // VectorXd        NS_MARGIN;
    // 
    // NS_Z2 = MatrixXd::Zero(7,1);    
    // NS_OPT_FCT = MatrixXd::Zero(1,1);    
    // NS_JACOB   = MatrixXd::Zero(1,7);    
    // NS_THETA_MEDIO    = VectorXd::Zero(var_ROBOT_DOFs); 
    // NS_MARGIN    = VectorXd::Zero(var_ROBOT_DOFs); 

    //  
    // for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {    
    //     NS_THETA_MEDIO(i) = 0.5*( var_ROBOT_JOINTS_LIM__UPPER(i) + var_ROBOT_JOINTS_LIM__LOWER(i) );
    //     NS_MARGIN(i) = 0.03*( var_ROBOT_JOINTS_LIM__UPPER(i) - var_ROBOT_JOINTS_LIM__LOWER(i) );
    // }        
    //*************************************************


    // bool should_break_loop = false;
    // while(not should_break_loop){

        // ******  Calculate JACOB (N = Hminus8(x_d) * C8 * J  )  ******

        //****** UPDATE ERROR  ********
        // var_task_kd_last_error   = var_task_ERROR;
        // var_task_ki_error = var_task_ERROR + var_ctrl_Ki_memorySize*var_task_ki_error;      

        // if (error.norm() < 0.1)
            error_integral = error + 0.99*error_integral;
        // else
        //     error_integral = 0.001*error;


        // //******  INVERSE MATRIX [init] *******  
        // // varJACOB.get_SVD(current_step_relevant_dof);
        // // varJACOB.get_INV(var_ctrl_sri_lambda, var_ctrl_srivar_lambda_max, var_ctrl_srivar_ballsize);
        // varJACOB.get_INV_left( var_ctrl_sri_lambda );       

        MatrixXd tIDENTIDADE;
        // jacobian_INV = get_jacobinv_left(jacobian, var_ctrl_sri_lambda);
        tIDENTIDADE = MatrixXd::Identity(14,14);     
        jacobian_INV =  ((  jacobian.transpose()*jacobian + (var_ctrl_sri_lambda)*(tIDENTIDADE)   ).inverse() )*jacobian.transpose();

        // tIDENTIDADE = MatrixXd::Identity(error.rows(),error.rows());     
        // jacobian_INV =  jacobian.transpose()*((  jacobian*jacobian.transpose() + (var_ctrl_sri_lambda)*(tIDENTIDADE)   ).inverse() );
 
        

        // //******  TRACKING ******
        // var_tracking_updateTerm  = MatrixXd::Zero(8,1);    
        // if (var_FLAG_TRACK_enable_tracking_term)  {
        //     if( var_FLAG_CTRL__at_least_one_reference )
        //       var_tracking_updateTerm = Hminus8( pose_Xd - var_tracking_lastReference )*C8()*vec8( pose_Xm );
        //     else
        //       var_FLAG_CTRL__at_least_one_reference = true;        
        //     var_tracking_lastReference = pose_Xd;            
        // }
    

        //******  OUTPUT CONTROL   ******
        // var_task_thetas_Delta = jacobian_INV*( 0.0105*error + 0.0001*error_integral );
        var_task_thetas_Delta = jacobian_INV*( 0.006*error + 0.00001*error_integral );
        // var_task_thetas_Delta = jacobian_INV*( 0.0075*error + 0*error_integral );
        return var_task_thetas_Delta;


        // if( var_FLAG_CTRL__at_least_one_error )
        //     varPSEUDO_ROBOT.thetas_delta = jacobian_INV*( var_ctrl_Kp*var_task_ERROR + var_ctrl_Ki*var_task_ki_error + var_ctrl_Kd*(var_task_ERROR - var_task_kd_last_error)  - var_tracking_updateTerm );            
        // else  {
        //   var_FLAG_CTRL__at_least_one_error = true;
        //   varPSEUDO_ROBOT.thetas_delta = jacobian_INV*( var_ctrl_Kp*var_task_ERROR + var_ctrl_Ki*var_task_ki_error  - var_tracking_updateTerm );
        // }


        // //**************************************************************************************************************[ TESTE SECTION ]
        // //**************************************************************************************************************[ TESTE SECTION ]
        // //**************************************************************************************************************[ TESTE SECTION ]
        // int NS_OPTIONS = 1;
        // //******  Null space optimization for joint limits   ******
        // varJACOB.get_NullSpaceProjector();
        // NS_OPT_FCT(0,0) = 0;
        // double nsGAIN= 0.5;
        
        // double xgain[7];
        // xgain[0]=8;
        // xgain[1]=6;
        // xgain[2]=4;
        // xgain[3]=4;
        // xgain[4]=2;
        // xgain[5]=1;
        // xgain[6]=1;

        // //*********************************===>   OPTION 1
        // if (NS_OPTIONS==1)
        // {
        //     for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {            
        //         NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + 0.5*xgain[i]*( varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) )*( varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) );
        //         NS_JACOB(0,i) =  xgain[i]*(varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) );
        //     }

        //     jacobtemp.jacob = NS_JACOB*( varJACOB.jacobNSproject );    
        //     // NS_Z2 =  dqjacob::get_INV( NS_JACOB*( varJACOB.jacobNSproject ), var_ctrl_sri_lambda )*(  var_ctrl_Kp*NS_OPT_FCT - 1*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );
        //     NS_Z2 = dqjacob::get_INV( jacobtemp.jacob, var_ctrl_sri_lambda )*(  0.025*NS_OPT_FCT - 0.5*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );

        //     // extra info
        //     // jacobtemp.get_SVD(jacobtemp.jacob.rows());
        //     // std::cout << "MIN SING VALUES: " << jacobtemp.singValues(0) << jacobtemp.singValues(1) << jacobtemp.singValues(2) << jacobtemp.singValues(3) << jacobtemp.singValues(4) << jacobtemp.singValues(5) << jacobtemp.singValues(6) << std::endl;
        //     // std::cout << "MIN SING VALUES: " << jacobtemp.singValues  << std::endl;
        //     // std::cout << "MIN SING VALUES: " << jacobtemp.singValues.size() << std::endl;
            
        //     // // NEW OUTPUT WITH NULL SPACE
        //     varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.0008*( varJACOB.jacobNSproject )*NS_Z2;
        // }


        // //*********************************===>   OPTION 2
        // if ( (NS_OPTIONS==2) || (NS_OPTIONS==3) )
        // {
        //     double jointslimdiv = 0;
        
        //     for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {  
        //         NS_JACOB(0,i)  = 0;   
        //         jointslimdiv  = varPSEUDO_ROBOT.thetas(i)-var_ROBOT_JOINTS_LIM__LOWER(i);
        //         if ( jointslimdiv  < NS_MARGIN(i) ) {
        //             NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + nsGAIN*0.5*pow(  (1/NS_MARGIN(i))*( NS_MARGIN(i)-jointslimdiv)  ,2); 
        //             NS_JACOB(0,i) =  -nsGAIN*(   (1/NS_MARGIN(i))*std::abs(  NS_MARGIN(i) - jointslimdiv )    ); 
        //             std::cout << "bottom: " << i << "=> (" << varPSEUDO_ROBOT.thetas(i) << " x " << var_ROBOT_JOINTS_LIM__LOWER(i)  << ")= "<< jointslimdiv <<" ==>  " <<  NS_JACOB(0,i) << std::endl;
        //         }
        //         else {
        //             jointslimdiv  = var_ROBOT_JOINTS_LIM__UPPER(i)-varPSEUDO_ROBOT.thetas(i);
        //             if ( jointslimdiv  < NS_MARGIN(i) ) {
        //                 NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + nsGAIN*0.5*pow(   (1/NS_MARGIN(i))*( NS_MARGIN(i)-jointslimdiv )    ,2); 
        //                 NS_JACOB(0,i) =  +nsGAIN*(  (1/NS_MARGIN(i))*std::abs( NS_MARGIN(i)-jointslimdiv )   ); 
        //                 std::cout << "top: " << i << "=> (" << varPSEUDO_ROBOT.thetas(i) << " x " << var_ROBOT_JOINTS_LIM__UPPER(i)  << ")= "<< jointslimdiv <<" ==>  " <<  NS_JACOB(0,i) << std::endl;
        //             }
        //             else
        //             {
        //                 NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0); 
        //                 NS_JACOB(0,i) =  0.00; 
        //             }
        //         }   
        //     }

        //     if (NS_OPTIONS==2) 
        //     {
        //         jacobtemp.jacob = NS_JACOB*( varJACOB.jacobNSproject );    
        //         NS_Z2 = dqjacob::get_INV( jacobtemp.jacob, var_ctrl_sri_lambda )*(  0.5*NS_OPT_FCT - 0.1*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );        

        //         // // NEW OUTPUT WITH NULL SPACE
        //         varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.5*( varJACOB.jacobNSproject )*NS_Z2;
        //         // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta ; // + 1*( varJACOB.jacobNSproject )*NS_Z2;
        //         // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.5*( varJACOB.jacobNSproject )*dqjacob::get_INV( NS_JACOB, 0 )*NS_OPT_FCT ;
        //     }
        //     else
        //     {
        //         // std::cout << "FCT: " << std::endl << NS_JACOB.transpose() << std::endl ;   
        //         // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta - (0.2)*NS_JACOB.transpose();     
        //         varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta - (2)*NS_JACOB.transpose();     
        //     }
        //     // extra info
        //     // jacobtemp.get_SVD(jacobtemp.jacob.rows());
        //     // std::cout << "jacob :   " << NS_OPT_FCT  << std::endl << jacobtemp.jacob << std::endl;
        //     // std::cout << "jacob inv:   "  <<  std::endl << varJACOB.jacob*dqjacob::get_INV( jacobtemp.jacob, 0 ) << std::endl;
        // }
        // if ( (NS_OPTIONS<0) || (NS_OPTIONS>3) )
        //     varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta;
        // // 
        // //**************************************************************************************************************[ TESTE SECTION ]
        // //**************************************************************************************************************[ TESTE SECTION ]
        //**************************************************************************************************************[ TESTE SECTION ]

        // //Update delta_thetas for FUTURE THETA calculation
        // for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {
        //     if(varPSEUDO_ROBOT.dummy_joints_marker(i) == 0) {
        //         var_task_thetas_Delta(i) = varPSEUDO_ROBOT.thetas_delta(j);
        //         ++j;
        //     }
        // }
        // ******  OUTPUT THETAS (POSSIBLE)   ******
        // varPSEUDO_ROBOT.thetas_output = var_task_thetas + var_task_thetas_Delta;

    // }//While not should_break_loop



}








//  bool Controller_Cooperative::joint_limit_verification_step() 
//  {
//     bool returnBreakLoop = true;
//     int j=0;
//     //For all joints
//     for(int i = 0; i < var_ROBOT_DOFs; i++){

//         //If joint is not yet marked for not being considered in the minimization
//         if(varPSEUDO_ROBOT.dummy_joints_marker(i) == 0){

//             //If the controller is trying to put a joint further than any of its limits
//             if(    varPSEUDO_ROBOT.thetas_output(i) > var_ROBOT_JOINTS_LIM__UPPER(i)
//                 || varPSEUDO_ROBOT.thetas_output(i) < var_ROBOT_JOINTS_LIM__LOWER(i) )
//             {

//                 //If the joint was already saturated sometime ago
//                 double ep = 1.e-05;
//                 if (    var_task_thetas(i) > var_ROBOT_JOINTS_LIM__UPPER(i) - ep 
//                      || var_task_thetas(i) < var_ROBOT_JOINTS_LIM__LOWER(i) + ep){

//                     varPSEUDO_ROBOT.dummy_joints_marker(i) = 1; //Mark it to be ignored in the minization
//                     //std::cout << std::endl << "Joint " << i << " will be ignored in the next controller step.";
//                     varPSEUDO_ROBOT.dh_matrix(4,i) = 1;          //Set matrix as dummy.
//                     varPSEUDO_ROBOT.dh_matrix(0,i) = var_task_thetas(i); //Set matrix theta as a fixed value.
//                     returnBreakLoop = false;
//                     if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
//                         std::cout << "[WARN]:[Eliminate Joint " << i <<"]: It has saturated its limits sometime ago. Value:" << var_task_thetas(i) << std::endl;
//                 }
//                 //If the joint was not yet saturated and the controller wants to saturate it
//                 else{
//                     // Saturate the joint in this step.
//                     if   ( varPSEUDO_ROBOT.thetas_output(i) > var_ROBOT_JOINTS_LIM__UPPER(i) ){
//                         var_task_thetas_Delta(i) = var_ROBOT_JOINTS_LIM__UPPER(i) - var_task_thetas(i);
//                         if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
//                             std::cout << "[WARN]: Joint (" << i <<") is going to saturate its limits. Value:" <<  varPSEUDO_ROBOT.thetas_output(i) << std::endl;
//                     }
//                     else if ( varPSEUDO_ROBOT.thetas_output(i) < var_ROBOT_JOINTS_LIM__LOWER(i) ){
//                         var_task_thetas_Delta(i) = var_ROBOT_JOINTS_LIM__LOWER(i) - var_task_thetas(i);
//                         if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
//                             std::cout << "[WARN]: Joint (" << i <<") is going to saturate its limits. Value:" <<  varPSEUDO_ROBOT.thetas_output(i) << std::endl;
//                     }
//                     else{
//                         std::cout << std::endl << "Something is really wrong";
//                     }
//                     //The joint should still be considered in the minimizations.
//                     varPSEUDO_ROBOT.thetas(j) = var_task_thetas(i);
//                     ++j;    
//                 }

  
//             }
//             //If the controller is not trying to put this joint further than any of its limits, we consider the velocity given normally
//             else{
//                 var_task_thetas_Delta(i) = varPSEUDO_ROBOT.thetas_delta(j);
//                 varPSEUDO_ROBOT.thetas(j) = var_task_thetas(i);
//                 ++j;      
//             }
//         }
//         //If joint was marked to be ignored, it shall be ignored.
//         else{
//             var_task_thetas_Delta(i) = 0;
//             if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
//                 std::cout << "[WARN]: Ignore move in Joint (" << i <<") due to joint limits" << std::endl;
//         }

//     }
//     if( j == 0 ){
//         std::cout << std::endl << "Robot will be unable to get out of this configuration using this controller.";
//         var_task_thetas_Delta = VectorXd::Zero(var_ROBOT_DOFs);
//         returnBreakLoop = true;
//         return returnBreakLoop;       
//     }
//     if(not returnBreakLoop){
//         varPSEUDO_ROBOT.KINE =  DQ_kinematics(varPSEUDO_ROBOT.dh_matrix);  //Change DH
//         varPSEUDO_ROBOT.KINE.set_base(var_ROBOT_KINE.base()); 
//         varPSEUDO_ROBOT.KINE.set_effector(var_ROBOT_KINE.effector());            
//         varPSEUDO_ROBOT.thetas.conservativeResize(j);           //Resize pseudothetas
//         if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
//             std::cout << "=========================================================" << std::endl;        
//     } 
//     return returnBreakLoop;     

//     std::cout << std::endl;           
     
// }




void Controller_Cooperative::enableTrackingTerm(const bool bool_falseortrue)
{
    var_FLAG_TRACK_enable_tracking_term = bool_falseortrue;
}








//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  SUBCLASS:    dqjacob          #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------




// /*********************************************************
// ##########################################################
// ####  SUBCLASS:   dqjacob                 
// ####  FUNCTION:   get_SVD : (int DOFs) -> void 
// ####______________________________________________________  
//     * Update the svd, singValues, min_singValue, min_U_svd using the jacobian from the subclass, and the input arg.
// ##########################################################
// **********************************************************/
// void Controller_Cooperative::dqjacob::get_SVD(int relevant_DOFs)
// {
//     svd.compute(jacob, ComputeFullU);
//     singValues    = svd.singularValues();  
//     min_singValue = singValues( relevant_DOFs - 1);
//     min_U_svd     = svd.matrixU().col( relevant_DOFs - 1);               
// }



// /*********************************************************
// ##########################################################
// ####  SUBCLASS:   dqjacob                 
// ####  FUNCTION:   get_INV : (double lambda) -> void 
// ####______________________________________________________  
//     * Calculate the Jacobian Inverse using SRI (sing robust inverse) with lamda
// ##########################################################
// **********************************************************/
// void Controller_Cooperative::dqjacob::get_INV( double lambda )
// {      
//     MatrixXd IDENTIDADE;
//     IDENTIDADE = Controller_Cooperative::getIDENTMATRIX_WITH_PROPER_SIZE(task_size); 
//     jacobInv =  (jacob.transpose())*((  jacob*jacob.transpose() 
//                     + (lambda)*(IDENTIDADE)   ).inverse() );
// }

// void Controller_Cooperative::dqjacob::get_INV_left( double lambda )
// {      
//     MatrixXd IDENTIDADE;
//     IDENTIDADE = Controller_Cooperative::getIDENTMATRIX_WITH_PROPER_SIZE(jacob.cols()); 

//     // Matrix<double,8,1> kp_diagonal(8,1);
//     // // kp_diagonal << 0.3,0.3,0.3,0.3,1,1,1,1;
//     // kp_diagonal << 1000,1000,1000,1000,1000,1000,1000,1;
//     // IDENTIDADE.diagonal() = kp_diagonal; 

//     jacobInv =  ((  jacob.transpose()*jacob + (lambda)*(IDENTIDADE)   ).inverse() )*jacob.transpose();
// }

// /// * Returns the inverse of a jacob_data using SRI (sing robust inverse) with lamda
// MatrixXd Controller_Cooperative::dqjacob::get_INV( MatrixXd jacob_data, double lambda )
// {      
//     MatrixXd IDENTIDADE;
//     MatrixXd outpumatrix;
//     IDENTIDADE = Controller_Cooperative::getIDENTMATRIX_WITH_PROPER_SIZE( jacob_data.rows()  ); 
//     // outpumatrix = jacob_data.transpose();    
//     outpumatrix = (jacob_data.transpose())*((  jacob_data*jacob_data.transpose() + (lambda)*(IDENTIDADE)   ).inverse() );
//     return  outpumatrix; 
// }




// /*********************************************************
// ##########################################################
// ####  SUBCLASS:   dqjacob                 
// ####  FUNCTION:   get_INV : (double lambda, srivar_lambda_max, srivar_lambda_region) -> void 
// ####______________________________________________________  
//     * Calculate the Jacobian Inverse using SRI-VAR (sing robust inverse) with lamda, lambda_var, lambda_var_region
// ##########################################################
// **********************************************************/
// void Controller_Cooperative::dqjacob::get_INV(double lambda, double srivar_lamda_max, double srivar_lamda_region )
// {
//     double      SRI_VAR_LAMBDA;
//     MatrixXd    IDENTIDADE;             
//     SRI_VAR_LAMBDA  = srivar_lamda_max;  
//     if (task_size==8)
//         IDENTIDADE = Matrix<double,8,8>::Identity();
//     else
//         IDENTIDADE = Matrix<double,4,4>::Identity();   

//     if (min_singValue < srivar_lamda_region)
//         SRI_VAR_LAMBDA = (  1-(min_singValue/srivar_lamda_region)*(min_singValue/srivar_lamda_region)  )*srivar_lamda_max;
//     // Get JACOBIAN INVERSE:
//     jacobInv =  (jacob.transpose())*((  jacob*jacob.transpose() 
//                     + (lambda)*(IDENTIDADE) 
//                     + (SRI_VAR_LAMBDA)*(min_U_svd*min_U_svd.transpose())  ).inverse() );
// }  
// ////// * Returns the inverse of a jacob_data using SRI-VAR with lamda, lambda_var, lambda_var_region






} // END OF NAMESPACE



     



