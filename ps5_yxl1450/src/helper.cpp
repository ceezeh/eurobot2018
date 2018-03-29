/*
 * helper.cpp
 *
 *  Created on: Feb 2, 2018
 *      Author: deeplearning
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "cppopt/optimization.h"
#include "ps5_yxl1450/helper.h"
using namespace alglib;
double goal[2];


double l1 = 1.1;
double l2 = 1.0;
double l3 = 1.0;
double gz = 0;
double gx = 0;
void nlcfunc1_jac(const real_1d_array &x, real_1d_array &fi, real_2d_array &jac,
		void *ptr) {
	//
	// this callback calculates
	//
	//     f0(x0,x1) = -x0+x1
	//     f1(x0,x1) = x0^2+x1^2-1
	//
	// and Jacobian matrix J = [dfi/dxj]
	//

	fi[0] = ((l1) - (gz) + sin(M_PI / 2 + (x[0]) + (x[1])) * (l3)
			+ sin(M_PI / 2 + (x[0])) * (l2))
			* (l1 - gz + l3 * sin(x[0] + x[1] + M_PI / 2) + l2 * sin(x[0] + M_PI / 2))
			+ (cos(M_PI / 2 + (x[0]) + (x[1])) * (l3) - (gx)
					+ cos(M_PI / 2 + (x[0])) * (l2))
					* (l3 * cos(x[0] + x[1] + M_PI / 2) - gx
							+ l2 * cos(x[0] + M_PI / 2)) + pow(x[0],2) / 1.2e1 + pow(x[1],2) / 1.1e2;

	jac[0][0] = ((l1) - (gz) + sin(M_PI / 2 + (x[0]) + (x[1])) * (l3)
			+ sin(M_PI / 2 + (x[0])) * (l2))
			* (l1 - gz + l3 * sin(x[0] + x[1] + M_PI / 2) + l2 * sin(x[0] + M_PI / 2))
			+ (cos(M_PI / 2 + (x[0]) + (x[1])) * (l3) - (gx)
					+ cos(M_PI / 2 + (x[0])) * (l2))
					* (l3 * cos(x[0] + x[1] + M_PI / 2) - gx
							+ l2 * cos(x[0] + M_PI / 2)) + pow(x[0],2) / 1.2e1 + pow(x[1],2) / 1.1e2;

	jac[0][1] = ((l1) - (gz)
			+ sin(M_PI / 2 + (x[0]) + (x[1])) * (l3)
			+ sin(M_PI / 2 + (x[0])) * (l2))
			* (l1 - gz + l3 * sin(x[0] + x[1] + M_PI / 2) + l2 * sin(x[0] + M_PI / 2))
			+ (cos(M_PI / 2 + (x[0]) + (x[1])) * (l3) - (gx)
					+ cos(M_PI / 2 + (x[0])) * (l2))
					* (l3 * cos(x[0] + x[1] + M_PI / 2) - gx
							+ l2 * cos(x[0] + M_PI / 2)) + pow(x[0],2) / 1.2e1 + pow(x[1],2) / 1.1e2;

}

std::vector<double> calcInvK(geometry_msgs::Point gripper_pos, double gripper_dist) {
	//
	// This example demonstrates minimization of
	//
	//     f(x0,x1) = -x0+x1
	//
	// subject to boundary constraints
	//
	//    x0>=0, x1>=0
	//
	// and nonlinear inequality constraint
	//
	//    x0^2 + x1^2 - 1 <= 0
	//
	double gripper_center = 0.15;
	double gripper_range = 0.16; // distance from the gripper paddle axis to gripper center
	double gripper_radius = 0.04; // the radius of the gripper paddle
	double half_dist = gripper_dist/2 + gripper_radius;
	gx = gripper_pos.x-gripper_center;
	gz = gripper_pos.z;

	real_1d_array x0 = "[-1.4, 0.6]";
	real_1d_array s = "[.5,.5]";
	double epsg = 0;
	double epsf = 1e-100;
	double epsx = 1e-100;
	ae_int_t maxits = 0;
	ae_int_t outerits = 5;
	ae_int_t updatefreq = 10;
	double rho = 1000;
	real_1d_array bndl = "[-1.5707963268,-1.5707963268]";
	real_1d_array bndu = "[0,0]";
	minnlcstate state;
	minnlcreport rep;
	real_1d_array x1;

	//
	// Create optimizer object, choose AUL algorithm and tune its settings:
	// * rho=1000       penalty coefficient
	// * outerits=5     number of outer iterations to tune Lagrange coefficients
	// * epsx=0.000001  stopM_M_PIng condition for inner iterations
	// * s=[1,1]        all variables have unit scale
	// * exact low-rank preconditioner is used, updated after each 10 iterations
	//
	minnlccreate(2, x0, state);
	minnlcsetalgoaul(state, rho, outerits);
	minnlcsetcond(state, epsg, epsf, epsx, maxits);
	minnlcsetscale(state, s);
	minnlcsetprecexactlowrank(state, updatefreq);

	//
	// Set constraints:
	//
	// 1. boundary constraints are passed with minnlcsetbc() call
	//
	// 2. nonlinear constraints are more tricky - you can not "pack" general
	//    nonlinear function into double precision array. That's why
	//    minnlcsetnlc() does not accept constraints itself - only constraint
	//    counts are passed: first parameter is number of equality constraints,
	//    second one is number of inequality constraints.
	//
	//    As for constraining functions - these functions are passed as part
	//    of problem Jacobian (see below).
	//
	// NOTE: MinNLC optimizer supports arbitrary combination of boundary, general
	//       linear and general nonlinear constraints. This example does not
	//       show how to work with general linear constraints, but you can
	//       easily find it in documentation on minnlcsetlc() function.
	//
	minnlcsetbc(state, bndl, bndu);
	minnlcsetnlc(state, 0, 0);

	//
	// Optimize and test results.
	//
	// Optimizer object accepts vector function and its Jacobian, with first
	// component (Jacobian row) being target function, and next components
	// (Jacobian rows) being nonlinear equality and inequality constraints.
	//
	// So, our vector function has form
	//
	//     {f0,f1} = { -x0+x1 , x0^2+x1^2-1 }
	//
	// with Jacobian
	//
	//         [  -1    +1  ]
	//     J = [            ]
	//         [ 2*x0  2*x1 ]
	//
	// with f0 being target function, f1 being constraining function. Number
	// of equality/inequality constraints is specified by minnlcsetnlc(),
	// with equality ones always being first, inequality ones being last.
	//
	alglib::minnlcoptimize(state, nlcfunc1_jac);
	minnlcresults(state, x1, rep);
	printf("%s\n", x1.tostring(2).c_str()); // EXPECTED: [1.0000,0.0000]

	// output joints vector
		std::vector<double> cmd_jnts;
		cmd_jnts.resize(6); // it's a 6 joints robot

		// start inverse kinematics
		// revolute joints: joint1 & 2 & 3 & 4
		double cyl_theta = atan2(gripper_pos.y, gripper_pos.x);
		cmd_jnts[0] = cyl_theta; // joint1, the easiest one...
		cmd_jnts[1] = -x1[0]; // joint2
		cmd_jnts[2] = -x1[1]; // joint3
		cmd_jnts[3] = (x1[1] +x1[0] +M_PI/2);// joint4, theta1 - theta2
		// prismatic joints: joint5 & 6
		cmd_jnts[4] = -(gripper_range - half_dist); // joint5
		cmd_jnts[5] = (gripper_range - half_dist); // joint6

		return cmd_jnts;

}
