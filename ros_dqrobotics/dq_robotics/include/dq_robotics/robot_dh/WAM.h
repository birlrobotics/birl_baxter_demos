/**
New WAM 7-DOF Robot DH Parameters
(without the hands)

*/

#ifndef DQ_ROBOTICS_WAM_DH_H
#define DQ_ROBOTICS_WAM_DH_H

#include"../DQ_kinematics.h"
#include<Eigen/Dense>
#include<cmath>

using namespace Eigen;

namespace DQ_robotics{

    DQ_kinematics WamKinematics()
    {
        const double pi2 = M_PI_2;

	    Matrix<double,5,7> wam_dh(5,7);
	    wam_dh << 0,    0,    0,      0,      0,   0,   0,
                  0,    0,    0.55,   0,      0.3, 0,   0.0609,
                  0,    0,    0.045, -0.045,  0,   0,   0,
                  -pi2, pi2, -pi2,    pi2,   -pi2, pi2, 0,
                  0,    0,    0,      0,      0,   0,   0;

	    DQ_kinematics wam(wam_dh,"standard");

        return wam;        
    };

}

#endif
