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
#include "grasp/helper.h"
using namespace alglib;
double goal[2];

double l1 = 1.1;
double l2 = 1.0;
double l3 = 1.0;
double gz = 0;
double gy = 0;
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
	double j02 = x[0];
	double j03 = x[1];
	double j12 = x[2];
	double j22 = x[3];
	double T1 = gx;
	double T2 = gy;
	double T3 = gz;
	fi[0] = (37 * sin(conj(j02)) + conj(T3) - 18) * (T3 + 37 * sin(j02) - 18)
			+ (j02 + j03 + j12 + j22 - pi / 2)
			^ 2 / 100
					+ (18 * cos(conj(j03) + conj(j12) + conj(j22))
							* cos(conj(j02)) - conj(T1)
							+ 11 * cos(conj(j02)) * cos(conj(j03))
							+ 8 * cos(conj(j02)) * cos(conj(j03) + conj(j12))
							+ 5)
							* (18 * cos(j03 + j12 + j22) * cos(j02) - T1
									+ 8 * cos(j03 + j12) * cos(j02)
									+ 11 * cos(j02) * cos(j03) + 5)
					+ (18 * cos(conj(j02))
							* sin(conj(j03) + conj(j12) + conj(j22)) - conj(T2)
							+ 11 * cos(conj(j02)) * sin(conj(j03))
							+ 8 * cos(conj(j02)) * sin(conj(j03) + conj(j12))
							- 15)
							* (11 * cos(j02) * sin(j03) - T2
									+ 18 * sin(j03 + j12 + j22) * cos(j02)
									+ 8 * sin(j03 + j12) * cos(j02) - 15) + j02
			^ 2 / 1000 + j03 ^ 2 / 1000 + j12 ^ 2 / 1000 + j22 ^ 2 / 1000;

	jac[0][0] = (11 * j02) / 500 + j03 / 50 + j12 / 50 + j22 / 50 - pi / 100
			- (11 * sin(j02) * sin(j03) + 18 * sin(j03 + j12 + j22) * sin(j02)
					+ 8 * sin(j03 + j12) * sin(j02))
					* (18 * cos(conj(j02))
							* sin(conj(j03) + conj(j12) + conj(j22)) - conj(T2)
							+ 11 * cos(conj(j02)) * sin(conj(j03))
							+ 8 * cos(conj(j02)) * sin(conj(j03) + conj(j12))
							- 15)
			- (18 * cos(conj(j03) + conj(j12) + conj(j22)) * sin(conj(j02))
					+ 11 * cos(conj(j03)) * sin(conj(j02))
					+ 8 * sin(conj(j02)) * cos(conj(j03) + conj(j12)))
					* (18 * cos(j03 + j12 + j22) * cos(j02) - T1
							+ 8 * cos(j03 + j12) * cos(j02)
							+ 11 * cos(j02) * cos(j03) + 5)
			+ 37 * cos(conj(j02)) * (T3 + 37 * sin(j02) - 18)
			- (8 * sin(conj(j02)) * sin(conj(j03) + conj(j12))
					+ 18 * sin(conj(j03) + conj(j12) + conj(j22))
							* sin(conj(j02))
					+ 11 * sin(conj(j02)) * sin(conj(j03)))
					* (11 * cos(j02) * sin(j03) - T2
							+ 18 * sin(j03 + j12 + j22) * cos(j02)
							+ 8 * sin(j03 + j12) * cos(j02) - 15)
			- (11 * cos(j03) * sin(j02) + 18 * cos(j03 + j12 + j22) * sin(j02)
					+ 8 * cos(j03 + j12) * sin(j02))
					* (18 * cos(conj(j03) + conj(j12) + conj(j22))
							* cos(conj(j02)) - conj(T1)
							+ 11 * cos(conj(j02)) * cos(conj(j03))
							+ 8 * cos(conj(j02)) * cos(conj(j03) + conj(j12))
							+ 5)
			+ 37 * cos(j02) * (37 * sin(conj(j02)) + conj(T3) - 18);

	jac[0][1] = j02 / 50 + (11 * j03) / 500 + j12 / 50 + j22 / 50 - pi / 100
			- (18 * cos(conj(j02)) * sin(conj(j03) + conj(j12) + conj(j22))
					+ 11 * cos(conj(j02)) * sin(conj(j03))
					+ 8 * cos(conj(j02)) * sin(conj(j03) + conj(j12)))
					* (18 * cos(j03 + j12 + j22) * cos(j02) - T1
							+ 8 * cos(j03 + j12) * cos(j02)
							+ 11 * cos(j02) * cos(j03) + 5)
			+ (18 * cos(conj(j03) + conj(j12) + conj(j22)) * cos(conj(j02))
					+ 11 * cos(conj(j02)) * cos(conj(j03))
					+ 8 * cos(conj(j02)) * cos(conj(j03) + conj(j12)))
					* (11 * cos(j02) * sin(j03) - T2
							+ 18 * sin(j03 + j12 + j22) * cos(j02)
							+ 8 * sin(j03 + j12) * cos(j02) - 15)
			- (11 * cos(j02) * sin(j03) + 18 * sin(j03 + j12 + j22) * cos(j02)
					+ 8 * sin(j03 + j12) * cos(j02))
					* (18 * cos(conj(j03) + conj(j12) + conj(j22))
							* cos(conj(j02)) - conj(T1)
							+ 11 * cos(conj(j02)) * cos(conj(j03))
							+ 8 * cos(conj(j02)) * cos(conj(j03) + conj(j12))
							+ 5)
			+ (18 * cos(j03 + j12 + j22) * cos(j02)
					+ 8 * cos(j03 + j12) * cos(j02) + 11 * cos(j02) * cos(j03))
					* (18 * cos(conj(j02))
							* sin(conj(j03) + conj(j12) + conj(j22)) - conj(T2)
							+ 11 * cos(conj(j02)) * sin(conj(j03))
							+ 8 * cos(conj(j02)) * sin(conj(j03) + conj(j12))
							- 15);

}

std::vector<double> calcInvK(geometry_msgs::Point gripper_pos) {
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
	gx = gripper_pos.x;
	gy = gripper_pos.y;
	gz = gripper_pos.z;

	real_1d_array x0 = "[0, 0, 0, 1.5707963268]";
	real_1d_array s = "[1,1, 1 , 1]";
	double epsg = 0;
	double epsf = 1e-100;
	double epsx = 1e-100;
	ae_int_t maxits = 0;
	ae_int_t outerits = 5;
	ae_int_t updatefreq = 10;
	double rho = 1000;
	real_1d_array bndl =
			"[-0.872664626, -1.5707963268, -1.5707963268,-1.5707963268]";
	real_1d_array bndu =
			"[0.872664626,0.436332313, 1.5707963268, 1.5707963268]]";
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
	printf("%s\n", x1.tostring(2).c_str());

	// output joints vector
	std::vector<double> cmd_jnts;
	cmd_jnts.resize(5); // it's a 6 joints robot

	// start inverse kinematics
	cmd_jnts[0] = x1[0];
	cmd_jnts[1] = x1[1];
	cmd_jnts[2] = x1[2];
	cmd_jnts[3] = x1[3];
	cmd_jnts[4] = -x1[0];

	return cmd_jnts;

}
