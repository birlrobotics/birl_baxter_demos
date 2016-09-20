/**
BAXTER - Left Arm - Robot DH Parameters

\author Luis Figueredo
\since 02/2014
*/

#ifndef DQ_ROBOTICS_BAXTER_ARM_DH_H
#define DQ_ROBOTICS_BAXTER_ARM_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics BaxterArm_Kinematics()
    {
        const double pi2 = M_PI_2;
        // const double endeffector_z = 0.229525;
        // const double endeffector_z = 0;
        // const double endeffector_z = 0.08229525;
        const double endeffector_z = 0.1145;
        // const double offtheta = 0; 
        const double offtheta = pi2; 

	    Matrix<double,5,7> baxter_arm_dh(5,7);      
	    baxter_arm_dh << 0,   offtheta,         0,       0,        0,      0,   0,
                  0.27035,      0,      0.36435,       0,    0.37429,    0,   endeffector_z,
                  0.069,        0,       0.0690,       0,     0.010,     0,   0,
                  -pi2,        pi2,        -pi2,     pi2,      -pi2,   pi2,   0,
                  0,            0,            0,       0,         0,     0,   0;      
      // baxter_arm_dh << 0,     pi2,          0,   0,        0,   0,   0,      
      //             0.27035,    0,    0.36435,   0,  0.37429,   0,   endeffector_z,
      //             0.069,      0,    0.0690,    0,    0.010,   0,   0,
      //             -pi2,     pi2,      -pi2,  pi2,   -pi2,   pi2,   0,
      //             0,       0,         0,    0,      0,   0,   0;

	    DQ_kinematics baxterArm(baxter_arm_dh,"standard");

        return baxterArm;        
    };

}

#endif 
// END OF DQ_ROBOTICS_BAXTER_ARM_DH_H
/*
Lr(1) = Link([0, 0.27035,   0.069, -pi/2, 0, 0], 'standard');
Lr(2) = Link ([0, 0,        0,      pi/2, 0, pi/2], 'standard');
Lr(3) = Link ([0, 0.36435,  0.0690, -pi/2, 0, 0], 'standard');
Lr(4) = Link ([0, 0,        0,      pi/2, 0, 0], 'standard');
Lr(5) = Link ([0, 0.37429,  0.010, -pi/2, 0, 0], 'standard');
Lr(6) = Link ([0, 0,        0,      pi/2, 0, 0], 'standard');
Lr(7) = Link ([0, 0.229525, 0,        0, 0, 0], 'standard');

*/
