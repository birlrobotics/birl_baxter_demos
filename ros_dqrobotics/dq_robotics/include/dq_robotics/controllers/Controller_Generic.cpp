#include "Controller_Generic.h"
// #include <Eigen/Dense>
// #include <iostream>

using namespace Eigen;


namespace DQ_robotics
{






                  
//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : Controller_Generic    #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  
/// * CALL init_default_parameters with robot input (DQ_kinematics)
Controller_Generic::Controller_Generic(DQ_kinematics robot) : DQ_controller()
{    
    init_default_parameters(robot);
    print_parameters();
}



/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Generic                 
####  FUNCTION:   init_default_parameters(DQ_kinematics robot) 
####_______________________________________________________________________________  
    * SET DEFAULT VALUES FOR THE PARAMETERS OF THE CLASS
####################################################################################    
***********************************************************************************/
void Controller_Generic::init_default_parameters(DQ_kinematics robot) 
{    
    //** DEBUG MODE:
    var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF = true;

    //** SIZE OF THE TASK
    var_TASK_SIZE = 8;

    ///** Constant variables
    var_const_DQ_1         = DQ(1);
    var_const_Identity_4x4 = Matrix<double,4,4>::Identity();
    var_const_Identity_8x8 = Matrix<double,8,8>::Identity();

    //** Init Flags   ****[ DEFAULT VALUES ]
    var_FLAG_CTRL__at_least_one_error     = false;    
    var_FLAG_CTRL__at_least_one_reference = false;
    var_FLAG_TRACK_enable_tracking_term   = false;
    var_FLAG_ROBOT__joint_limits          = false;


    //** Init Robotic Arm Param
    var_ROBOT_DOFs     = (robot.links() - robot.n_dummy());
    var_ROBOT_KINE     = robot;
    var_ROBOT_JOINTS_LIM__UPPER    = VectorXd::Zero(var_ROBOT_DOFs); // upper_joint_limits;
    var_ROBOT_JOINTS_LIM__LOWER    = VectorXd::Zero(var_ROBOT_DOFs); // lower_joint_limits;
    var_ROBOT_DUMMY_JOINTS         = var_ROBOT_KINE.dummy(); 

    //** Control Param ****[ DEFAULT VALUES ]
    var_ctrl_Kp     = MatrixXd::Identity(8,8);
    var_ctrl_Kp     = 0.1*var_ctrl_Kp;
    var_ctrl_Ki     = MatrixXd::Zero(8,8);
    var_ctrl_Kd     = MatrixXd::Zero(8,8);
    var_ctrl_Ki_memorySize = 0.99;
    var_ctrl_sri_lambda       = 0.01;
    var_ctrl_srivar_lambda_max = 0.0; //0.05;
    var_ctrl_srivar_ballsize  = 0.001;

    //** Init evolution Task variables
    var_task_thetas         = MatrixXd(var_ROBOT_DOFs,1);
    var_task_thetas_Delta   = MatrixXd::Zero(var_ROBOT_DOFs,1);        
    var_task_ERROR          = MatrixXd(8,1);
    var_task_ki_error       = MatrixXd::Zero(8,1);
    var_task_kd_last_error  = MatrixXd::Zero(8,1);

    // // TASK JACOB 
    varJACOB  = dqjacob(var_ROBOT_DOFs, 8);
    varJACOB4 = dqjacob(var_ROBOT_DOFs, 4);


    //** Tracking Terms
    var_tracking_updateTerm      = MatrixXd::Zero(8,1);    
    var_tracking_lastReference   = DQ(0,0,0,0,0,0,0,0);    
    var_tracking4_updateTerm      = MatrixXd::Zero(4,1);    
    var_tracking4_lastReference   = DQ(0,0,0,0);   
}







                  
//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : Controller_Generic    #########################################
//#################################         CONFIGURATION          #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  


/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Generic                 
####  FUNCTION:   set_joint_limits (VectorXd:  upper_limits and lower_limits )
####_______________________________________________________________________________  
    * SET the arm joint limits and enable joint limits exclusion
***********************************************************************************/
void Controller_Generic::set_joint_limits(VectorXd upper_joint_limits, VectorXd lower_joint_limits)
{
    var_FLAG_ROBOT__joint_limits  = true;
    var_ROBOT_JOINTS_LIM__UPPER   = upper_joint_limits;
    var_ROBOT_JOINTS_LIM__LOWER   = lower_joint_limits;
}



/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Generic                 
####  FUNCTION:   set_control_gains (different options)
####_______________________________________________________________________________  
    * Adjust control gains of the PID (just P matrix)
***********************************************************************************/
void Controller_Generic::set_control_gains(const MatrixXd& kp){
    var_ctrl_Kp             = kp;
}
/// * Adjust control gains of the PID (just Kp and Ki matrices)
void Controller_Generic::set_control_gains(const MatrixXd& kp, const MatrixXd& ki){
    var_ctrl_Kp             = kp;
    var_ctrl_Ki             = ki;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd matrices)
void Controller_Generic::set_control_gains(const MatrixXd& kp, const MatrixXd& ki, const MatrixXd& kd){
    set_control_gains(kp, ki);
    var_ctrl_Kd             = kd;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd matrices | and  integral memory it)
void Controller_Generic::set_control_gains(const MatrixXd& kp, const MatrixXd& ki, const double ki_memory, const MatrixXd& kd){
    set_control_gains(kp, ki, kd);
    var_ctrl_Ki_memorySize  = ki_memory;   
}
/// * Adjust control gains of the PID (just P gain-scalar)
void Controller_Generic::set_control_gains(double kp){
    MatrixXd IDENTIDADE;
    IDENTIDADE = get_IDENT();
    var_ctrl_Kp             = kp*IDENTIDADE;
}
/// * Adjust control gains of the PID (just Kp and Ki gain-scalars)
void Controller_Generic::set_control_gains(double kp, double ki){
    MatrixXd IDENTIDADE;
    IDENTIDADE = get_IDENT();
    var_ctrl_Kp             = kp*IDENTIDADE;
    var_ctrl_Ki             = ki*IDENTIDADE;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd gain-scalars)
void Controller_Generic::set_control_gains(double kp, double ki, double kd){
    MatrixXd IDENTIDADE;
    IDENTIDADE = get_IDENT();
    var_ctrl_Kp             = kp*IDENTIDADE;
    var_ctrl_Ki             = ki*IDENTIDADE;
    var_ctrl_Kd             = kd*IDENTIDADE;
}
/// * Adjust control gains of the PID (Kp and Ki and Kd gain-scalars | and  integral memory it)
void Controller_Generic::set_control_gains(double kp, double ki, const double ki_memory, double kd){
    set_control_gains(kp, ki, kd);
    var_ctrl_Ki_memorySize  = ki_memory;   
}


/**********************************************************************************
####################################################################################
####  CLASS:   Controller_Generic                 
####  FUNCTION:   set_jacob_srivar_paramconst (double:  sri_lambda, srivar_lambda_max, sri_var_lambda_region)
####_______________________________________________________________________________  
    * Adjust damping parameters for the SRI variable inverse  
####################################################################################    
***********************************************************************************/
void Controller_Generic::set_jacob_srivar_paramconst(const double& sri_lambda, const double& sriVar_lambda_max, const double& sriVar_ball_size)
{
    var_ctrl_sri_lambda          = sri_lambda;
    var_ctrl_srivar_lambda_max   = sriVar_lambda_max;
    var_ctrl_srivar_ballsize     = sriVar_ball_size;
}
/// * Adjust damping parameter for the SRI  inverse  
void Controller_Generic::set_jacob_srivar_paramconst(const double& sri_lambda)
{
    var_ctrl_sri_lambda          = sri_lambda;
}
//***************************************************************************************************************


void Controller_Generic::print_parameters()
{

}






//----------------------------------------------------------------------------------------------------------
//##########################################################################################################
//#################################================================#########################################
//#################################                                #########################################
//#################################  CLASS : Controller_Generic    #########################################
//#################################         CONTROLLERS            #########################################
//#################################                                #########################################
//#################################================================#########################################
//##########################################################################################################
//----------------------------------------------------------------------------------------------------------  
VectorXd Controller_Generic::getNewJointPositions( const DQ reference, const VectorXd thetas)
{
    var_task_thetas_Delta = getNewJointVelocities(reference, thetas, 1.0);
    // Send updated thetas to simulation
    return (var_task_thetas + var_task_thetas_Delta);
}

VectorXd Controller_Generic::getNewJointPositions( const DQ reference, const VectorXd thetas, double POS_GAIN)
{
    var_task_thetas_Delta = getNewJointVelocities(reference, thetas, POS_GAIN);
    // Send updated thetas to simulation
    return (var_task_thetas + var_task_thetas_Delta);
}
VectorXd Controller_Generic::getNewJointVelocities( const DQ reference, const VectorXd thetas)
{
    return getNewJointVelocities( reference, thetas, 1.0);     
}


VectorXd Controller_Generic::getNewJointVelocities( const DQ reference, const VectorXd thetas, double POS_GAIN)
{
    var_task_thetas = thetas; //This is necessary for the getNewJointPositions to work

    //** Variables for Joint Limit Verification
    varPSEUDO_ROBOT = pseudoRobot();
    varPSEUDO_ROBOT.setData(var_ROBOT_KINE, var_ROBOT_DOFs, var_task_thetas);

    //** Local Control Variables
    DQ pose_Xd;
    DQ pose_Xm;
    DQ end_effector_pose_ = DQ(0,0,0,0,0,0,0,0);
    DQ pose_reference = reference;

    MatrixXd mtemp;
    Matrix<double,8,1> mtemp8;
    
    mtemp8 = pose_reference.vec8();
    pose_reference = pose_reference*(pose_reference.norm().inv());
    pose_Xd = DQ( mtemp8(0,0), mtemp8(1,0), mtemp8(2,0), mtemp8(3,0), 0, 0, 0,0);    
    mtemp8 = pose_reference.translation().vec8();
    pose_Xd =  pose_Xd + POS_GAIN*0.5*E_*DQ( 0, mtemp8(1,0), mtemp8(2,0), mtemp8(3,0), 0, 0, 0,0 )*pose_Xd;

    
    // NULL SPACE VARIABLES 
    //*************************************************
    dqjacob jacobtemp;     //  7 x 1
    MatrixXd NS_Z2;     //  7 x 1
    MatrixXd NS_OPT_FCT; // n x 1
    MatrixXd NS_JACOB;  //  n x 7
    VectorXd        NS_THETA_MEDIO;
    VectorXd        NS_MARGIN;

    jacobtemp  = dqjacob(var_ROBOT_DOFs, 8);


    NS_Z2 = MatrixXd::Zero(7,1);    
    NS_OPT_FCT = MatrixXd::Zero(1,1);    
    NS_JACOB   = MatrixXd::Zero(1,7);    
    NS_THETA_MEDIO    = VectorXd::Zero(var_ROBOT_DOFs); 
    NS_MARGIN    = VectorXd::Zero(var_ROBOT_DOFs); 

    for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {    
        NS_THETA_MEDIO(i) = 0.5*( var_ROBOT_JOINTS_LIM__UPPER(i) + var_ROBOT_JOINTS_LIM__LOWER(i) );
        NS_MARGIN(i) = 0.03*( var_ROBOT_JOINTS_LIM__UPPER(i) - var_ROBOT_JOINTS_LIM__LOWER(i) );
    }    
    
    //*************************************************



    bool should_break_loop = false;
    while(not should_break_loop){

        //******  Calculate JACOB (N = Hminus8(x_d) * C8 * J  )  ******
        varJACOB.jacob = varPSEUDO_ROBOT.KINE.analyticalJacobian(varPSEUDO_ROBOT.thetas);
        varJACOB.jacob =  Hminus8(pose_Xd)*C8()*varJACOB.jacob;

        //****** Current POSE (given pseudo robot) ******
        end_effector_pose_ = varPSEUDO_ROBOT.KINE.fkm(varPSEUDO_ROBOT.thetas);
        mtemp8 = end_effector_pose_.vec8();
        pose_Xm = DQ( mtemp8(0,0), mtemp8(1,0), mtemp8(2,0), mtemp8(3,0), 0, 0, 0,0);
        end_effector_pose_ = end_effector_pose_*(end_effector_pose_.norm().inv()); 
        mtemp8 = end_effector_pose_.translation().vec8();
        pose_Xm =  pose_Xm + POS_GAIN*0.5*E_*DQ( 0, mtemp8(1,0), mtemp8(2,0), mtemp8(3,0), 0, 0, 0,0)*pose_Xm;        


        //****** UPDATE ERROR  ********
        var_task_kd_last_error   = var_task_ERROR;
        var_task_ERROR           = vec8( var_const_DQ_1 - (pose_Xm.conj())*pose_Xd );
        var_task_ki_error = var_task_ERROR + var_ctrl_Ki_memorySize*var_task_ki_error;        




        //******  INVERSE MATRIX [init] *******  
        int current_step_relevant_dof = ( varPSEUDO_ROBOT.KINE.links() - varPSEUDO_ROBOT.KINE.n_dummy() );
        varJACOB.get_SVD(current_step_relevant_dof);
        // varJACOB.get_INV(var_ctrl_sri_lambda, var_ctrl_srivar_lambda_max, var_ctrl_srivar_ballsize);
        varJACOB.get_INV_left( var_ctrl_sri_lambda );


        //******  TRACKING ******
        var_tracking_updateTerm  = MatrixXd::Zero(8,1);    
        if (var_FLAG_TRACK_enable_tracking_term)  {
            if( var_FLAG_CTRL__at_least_one_reference )
              var_tracking_updateTerm = Hminus8( pose_Xd - var_tracking_lastReference )*C8()*vec8( pose_Xm );
            else
              var_FLAG_CTRL__at_least_one_reference = true;        
            var_tracking_lastReference = pose_Xd;            
        }
    
        //******  OUTPUT CONTROL   ******
        if( var_FLAG_CTRL__at_least_one_error )
            varPSEUDO_ROBOT.thetas_delta = varJACOB.jacobInv*( var_ctrl_Kp*var_task_ERROR + var_ctrl_Ki*var_task_ki_error + var_ctrl_Kd*(var_task_ERROR - var_task_kd_last_error)  - var_tracking_updateTerm );            
        else  {
          var_FLAG_CTRL__at_least_one_error = true;
          varPSEUDO_ROBOT.thetas_delta = varJACOB.jacobInv*( var_ctrl_Kp*var_task_ERROR + var_ctrl_Ki*var_task_ki_error  - var_tracking_updateTerm );
        }

        //**************************************************************************************************************[                  ]
        //**************************************************************************************************************[ NULL SPACE       ]
        //**************************************************************************************************************[  EXPERIMENTATION ]
        int NS_OPTIONS = 0;
        //******  Null space optimization for joint limits   ******

        if (NS_OPTIONS > 0)        
            varJACOB.get_NullSpaceProjector();

        NS_OPT_FCT(0,0) = 0;
        double nsGAIN= 0.5;   

        //*********************************===>   OPTION 1
        if (NS_OPTIONS==1)
        {                 
            double xgain[7];
            xgain[0]=8;            xgain[1]=6;
            xgain[2]=4;            xgain[3]=4;
            xgain[4]=2;            xgain[5]=1;
            xgain[6]=1;            
            for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {            
                NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + 0.5*xgain[i]*( varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) )*( varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) );
                NS_JACOB(0,i) =  xgain[i]*(varPSEUDO_ROBOT.thetas(i)-NS_THETA_MEDIO(i) );
            }

            jacobtemp.jacob = NS_JACOB*( varJACOB.jacobNSproject );    
            // NS_Z2 =  dqjacob::get_INV( NS_JACOB*( varJACOB.jacobNSproject ), var_ctrl_sri_lambda )*(  var_ctrl_Kp*NS_OPT_FCT - 1*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );
            NS_Z2 = dqjacob::get_INV( jacobtemp.jacob, var_ctrl_sri_lambda )*(  0.025*NS_OPT_FCT - 0.5*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );
            
            // // NEW OUTPUT WITH NULL SPACE
            // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.0008*( varJACOB.jacobNSproject )*NS_Z2;
            // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.0001*( varJACOB.jacobNSproject )*NS_Z2;
            varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.000025*( varJACOB.jacobNSproject )*NS_Z2;
        }


        //*********************************===>   OPTION 2
        if ( (NS_OPTIONS==2) || (NS_OPTIONS==3) )
        {
            double jointslimdiv = 0;
        
            for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {  
                NS_JACOB(0,i)  = 0;   
                jointslimdiv  = varPSEUDO_ROBOT.thetas(i)-var_ROBOT_JOINTS_LIM__LOWER(i);
                if ( jointslimdiv  < NS_MARGIN(i) ) {
                    NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + nsGAIN*0.5*pow(  (1/NS_MARGIN(i))*( NS_MARGIN(i)-jointslimdiv)  ,2); 
                    NS_JACOB(0,i) =  -nsGAIN*(   (1/NS_MARGIN(i))*std::abs(  NS_MARGIN(i) - jointslimdiv )    ); 
                    std::cout << "bottom: " << i << "=> (" << varPSEUDO_ROBOT.thetas(i) << " x " << var_ROBOT_JOINTS_LIM__LOWER(i)  << ")= "<< jointslimdiv <<" ==>  " <<  NS_JACOB(0,i) << std::endl;
                }
                else {
                    jointslimdiv  = var_ROBOT_JOINTS_LIM__UPPER(i)-varPSEUDO_ROBOT.thetas(i);
                    if ( jointslimdiv  < NS_MARGIN(i) ) {
                        NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0) + nsGAIN*0.5*pow(   (1/NS_MARGIN(i))*( NS_MARGIN(i)-jointslimdiv )    ,2); 
                        NS_JACOB(0,i) =  +nsGAIN*(  (1/NS_MARGIN(i))*std::abs( NS_MARGIN(i)-jointslimdiv )   ); 
                        std::cout << "top: " << i << "=> (" << varPSEUDO_ROBOT.thetas(i) << " x " << var_ROBOT_JOINTS_LIM__UPPER(i)  << ")= "<< jointslimdiv <<" ==>  " <<  NS_JACOB(0,i) << std::endl;
                    }
                    else
                    {
                        NS_OPT_FCT(0,0) = NS_OPT_FCT(0,0); 
                        NS_JACOB(0,i) =  0.00; 
                    }
                }   
            }

            if (NS_OPTIONS==2) 
            {
                jacobtemp.jacob = NS_JACOB*( varJACOB.jacobNSproject );    
                NS_Z2 = dqjacob::get_INV( jacobtemp.jacob, var_ctrl_sri_lambda )*(  0.5*NS_OPT_FCT - 0.1*NS_JACOB*varPSEUDO_ROBOT.thetas_delta );        

                // // NEW OUTPUT WITH NULL SPACE
                varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.5*( varJACOB.jacobNSproject )*NS_Z2;
                // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta ; // + 1*( varJACOB.jacobNSproject )*NS_Z2;
                // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta + 0.5*( varJACOB.jacobNSproject )*dqjacob::get_INV( NS_JACOB, 0 )*NS_OPT_FCT ;
            }
            else
            {
                // std::cout << "FCT: " << std::endl << NS_JACOB.transpose() << std::endl ;   
                // varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta - (0.2)*NS_JACOB.transpose();     
                varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta - (2)*NS_JACOB.transpose();     
            }
            // extra info
            // jacobtemp.get_SVD(jacobtemp.jacob.rows());
        }
        if ( (NS_OPTIONS<0) || (NS_OPTIONS>3) )
            varPSEUDO_ROBOT.thetas_delta  =  varPSEUDO_ROBOT.thetas_delta;



        // 
        //**************************************************************************************************************[               ]
        //**************************************************************************************************************[    SECTION    ]
        //**************************************************************************************************************[       END     ]

        //Update delta_thetas for FUTURE THETA calculation
        for(int i=0,j=0; i < var_ROBOT_DOFs; i++)   {
            if(varPSEUDO_ROBOT.dummy_joints_marker(i) == 0) {
                var_task_thetas_Delta(i) = varPSEUDO_ROBOT.thetas_delta(j);
                ++j;
            }
        }


        //******  OUTPUT THETAS (POSSIBLE)   ******
        varPSEUDO_ROBOT.thetas_output = var_task_thetas + var_task_thetas_Delta;

        // //Verify if loop should end
        should_break_loop = true;
        if (NS_OPTIONS==0)
        {
            if (var_FLAG_ROBOT__joint_limits)
                should_break_loop = joint_limit_verification_step();  
            else
                should_break_loop = true;
        }

    }//While not should_break_loop

    return var_task_thetas_Delta;

}








 bool Controller_Generic::joint_limit_verification_step() 
 {
    bool returnBreakLoop = true;
    int j=0;
    //For all joints
    for(int i = 0; i < var_ROBOT_DOFs; i++){

        //If joint is not yet marked for not being considered in the minimization
        if(varPSEUDO_ROBOT.dummy_joints_marker(i) == 0){

            //If the controller is trying to put a joint further than any of its limits
            if(    varPSEUDO_ROBOT.thetas_output(i) > var_ROBOT_JOINTS_LIM__UPPER(i)
                || varPSEUDO_ROBOT.thetas_output(i) < var_ROBOT_JOINTS_LIM__LOWER(i) )
            {
                //If the joint speed (delta) acts as to push the joint away from its limits
                //  we consider the velocity given normally
                if (   (varPSEUDO_ROBOT.thetas_output(i) > var_ROBOT_JOINTS_LIM__UPPER(i)  &&  var_task_thetas_Delta(i) < 0)
                    || (varPSEUDO_ROBOT.thetas_output(i) < var_ROBOT_JOINTS_LIM__LOWER(i) &&  var_task_thetas_Delta(i) > 0) )       
                {
                    var_task_thetas_Delta(i) = varPSEUDO_ROBOT.thetas_delta(j);
                    varPSEUDO_ROBOT.thetas(j) = var_task_thetas(i);
                    ++j;    
                    continue;
                }

                //If the joint was already saturated sometime ago
                double ep = 1.e-05;
                if (    var_task_thetas(i) > var_ROBOT_JOINTS_LIM__UPPER(i) - ep 
                     || var_task_thetas(i) < var_ROBOT_JOINTS_LIM__LOWER(i) + ep){

                    varPSEUDO_ROBOT.dummy_joints_marker(i) = 1; //Mark it to be ignored in the minization
                    //std::cout << std::endl << "Joint " << i << " will be ignored in the next controller step.";
                    varPSEUDO_ROBOT.dh_matrix(4,i) = 1;          //Set matrix as dummy.
                    varPSEUDO_ROBOT.dh_matrix(0,i) = var_task_thetas(i); //Set matrix theta as a fixed value.
                    returnBreakLoop = false;
                    if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
                        std::cout << "[WARN]:[Eliminate Joint " << i <<"]: It has saturated its limits sometime ago. Value:" << var_task_thetas(i) << std::endl;
                }
                //If the joint was not yet saturated and the controller wants to saturate it
                else{
                    // Saturate the joint in this step.
                    if   ( varPSEUDO_ROBOT.thetas_output(i) > var_ROBOT_JOINTS_LIM__UPPER(i) ){
                        var_task_thetas_Delta(i) = var_ROBOT_JOINTS_LIM__UPPER(i) - var_task_thetas(i);
                        if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
                            std::cout << "[WARN]: Joint (" << i <<") is going to saturate its upper limits. Value:" <<  varPSEUDO_ROBOT.thetas_output(i) << std::endl;
                    }
                    else if ( varPSEUDO_ROBOT.thetas_output(i) < var_ROBOT_JOINTS_LIM__LOWER(i) ){
                        var_task_thetas_Delta(i) = var_ROBOT_JOINTS_LIM__LOWER(i) - var_task_thetas(i);
                        if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
                            std::cout << "[WARN]: Joint (" << i <<") is going to saturate its lower limits. Value:" <<  varPSEUDO_ROBOT.thetas_output(i) << std::endl;
                    }
                    else{
                        std::cout << std::endl << "Something is really wrong";
                    }
                    //The joint should still be considered in the minimizations.
                    varPSEUDO_ROBOT.thetas(j) = var_task_thetas(i);
                    ++j;    
                }

  
            }
            //If the controller is not trying to put this joint further than any of its limits, we consider the velocity given normally
            else{
                var_task_thetas_Delta(i) = varPSEUDO_ROBOT.thetas_delta(j);
                varPSEUDO_ROBOT.thetas(j) = var_task_thetas(i);
                ++j;      
            }
        }
        //If joint was marked to be ignored, it shall be ignored.
        else{
            var_task_thetas_Delta(i) = 0;
            if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
                std::cout << "[WARN]: Ignore move in Joint (" << i <<") due to joint limits" << std::endl;
        }

    }
    if( j == 0 ){
        std::cout << std::endl << "Robot will be unable to get out of this configuration using this controller.";
        var_task_thetas_Delta = VectorXd::Zero(var_ROBOT_DOFs);
        returnBreakLoop = true;
        return returnBreakLoop;       
    }
    if(not returnBreakLoop){
        varPSEUDO_ROBOT.KINE =  DQ_kinematics(varPSEUDO_ROBOT.dh_matrix);  //Change DH
        varPSEUDO_ROBOT.KINE.set_base(var_ROBOT_KINE.base()); 
        varPSEUDO_ROBOT.KINE.set_effector(var_ROBOT_KINE.effector());            
        varPSEUDO_ROBOT.thetas.conservativeResize(j);           //Resize pseudothetas
        if (var_FLAG_DEBUG__MODE_JOINTLIMIT_VERIF)
            std::cout << "=========================================================" << std::endl;        
    } 
    return returnBreakLoop;     

    std::cout << std::endl;           
     
}




void Controller_Generic::enableTrackingTerm(const bool bool_falseortrue)
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




/*********************************************************
##########################################################
####  SUBCLASS:   dqjacob                 
####  FUNCTION:   get_SVD : (int DOFs) -> void 
####______________________________________________________  
    * Update the svd, singValues, min_singValue, min_U_svd using the jacobian from the subclass, and the input arg.
##########################################################
**********************************************************/
void Controller_Generic::dqjacob::get_SVD(int relevant_DOFs)
{
    svd.compute(jacob, ComputeFullU);
    singValues    = svd.singularValues();  
    min_singValue = singValues( relevant_DOFs - 1);
    min_U_svd     = svd.matrixU().col( relevant_DOFs - 1);               
}



/*********************************************************
##########################################################
####  SUBCLASS:   dqjacob                 
####  FUNCTION:   get_INV : (double lambda) -> void 
####______________________________________________________  
    * Calculate the Jacobian Inverse using SRI (sing robust inverse) with lamda
##########################################################
**********************************************************/
void Controller_Generic::dqjacob::get_INV( double lambda )
{      
    MatrixXd IDENTIDADE;
    IDENTIDADE = Controller_Generic::getIDENTMATRIX_WITH_PROPER_SIZE(task_size); 
    jacobInv =  (jacob.transpose())*((  jacob*jacob.transpose() 
                    + (lambda)*(IDENTIDADE)   ).inverse() );
}

void Controller_Generic::dqjacob::get_INV_left( double lambda )
{      
    MatrixXd IDENTIDADE;
    IDENTIDADE = Controller_Generic::getIDENTMATRIX_WITH_PROPER_SIZE(jacob.cols()); 
    jacobInv =  ((  jacob.transpose()*jacob + (lambda)*(IDENTIDADE)   ).inverse() )*jacob.transpose();
}

/// * Returns the inverse of a jacob_data using SRI (sing robust inverse) with lamda
MatrixXd Controller_Generic::dqjacob::get_INV( MatrixXd jacob_data, double lambda )
{      
    MatrixXd IDENTIDADE;
    MatrixXd outpumatrix;
    IDENTIDADE = Controller_Generic::getIDENTMATRIX_WITH_PROPER_SIZE( jacob_data.rows()  ); 
    // outpumatrix = jacob_data.transpose();    
    outpumatrix = (jacob_data.transpose())*((  jacob_data*jacob_data.transpose() + (lambda)*(IDENTIDADE)   ).inverse() );
    return  outpumatrix; 
}




/*********************************************************
##########################################################
####  SUBCLASS:   dqjacob                 
####  FUNCTION:   get_INV : (double lambda, srivar_lambda_max, srivar_lambda_region) -> void 
####______________________________________________________  
    * Calculate the Jacobian Inverse using SRI-VAR (sing robust inverse) with lamda, lambda_var, lambda_var_region
##########################################################
**********************************************************/
void Controller_Generic::dqjacob::get_INV(double lambda, double srivar_lamda_max, double srivar_lamda_region )
{
    double      SRI_VAR_LAMBDA;
    MatrixXd    IDENTIDADE;             
    SRI_VAR_LAMBDA  = srivar_lamda_max;  
    if (task_size==8)
        IDENTIDADE = Matrix<double,8,8>::Identity();
    else
        IDENTIDADE = Matrix<double,4,4>::Identity();   

    if (min_singValue < srivar_lamda_region)
        SRI_VAR_LAMBDA = (  1-(min_singValue/srivar_lamda_region)*(min_singValue/srivar_lamda_region)  )*srivar_lamda_max;
    // Get JACOBIAN INVERSE:
    jacobInv =  (jacob.transpose())*((  jacob*jacob.transpose() 
                    + (lambda)*(IDENTIDADE) 
                    + (SRI_VAR_LAMBDA)*(min_U_svd*min_U_svd.transpose())  ).inverse() );
}  
////// * Returns the inverse of a jacob_data using SRI-VAR with lamda, lambda_var, lambda_var_region






} // END OF NAMESPACE



     



