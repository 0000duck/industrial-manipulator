/*
 * test.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

# include "math/Rotation3D.h"
# include "math/HTransform3D.h"
# include "kinematics/Trsf.h"
# include "kinematics/Frame.h"
# include <iostream>
# include <math.h>
# include <string>
# include "./common/printAdvance.h"
//# include "model/SerialLink.h"
# include "model/Link.h"
# include "test.h"
# include "ik/PieperSolver.h"

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;

int main(){
	cout<<"test";
	SerialLink robot;
	double alpha1 = M_PI/2;
	double alpha2 = 0;
	double alpha3 = M_PI/2;
	double alpha4 = -M_PI/2;
	double alpha5 = M_PI/2;
	double alpha6 = 0;
	double a1 = 40;
	double a2 = 315;
	double a3 = 70;
	double a4 = 0;
	double a5 = 0;
	double a6 = 0;
	double d1 = 330;
	double d2 = 0;
	double d3 = 310;
	double d4 = 0;
	double d5 = 0;
	double d6 = 70;
	double theta1 = 0;
	double theta2 = M_PI/2;
	double theta3 = 0;
	double theta4 = 0;
	double theta5 = -M_PI;
	double theta6 = M_PI/2;
	double lmin1 = M_PI/180.0*-180;
	double lmin2 = M_PI/180.0*-130;
	double lmin3 = M_PI/180.0*-70;
	double lmin4 = M_PI/180.0*-240;
	double lmin5 = M_PI/180.0*-30;
	double lmin6 = M_PI/180.0*-360;
	double lmax1 = M_PI/180.0*180;
	double lmax2 = M_PI/180.0*80;
	double lmax3 = M_PI/180.0*160;
	double lmax4 = M_PI/180.0*240;
	double lmax5 = M_PI/180.0*200;
	double lmax6 = M_PI/180.0*360;
//	Link(alpha, a, d, theta, min, max, sigma=0)
	Link l1(alpha1, a1, d1, theta1, lmin1, lmax1);
	Link l2(alpha2, a2, d2, theta2, lmin2, lmax2);
	Link l3(alpha3, a3, d3, theta3, lmin3, lmax3);
	Link l4(alpha4, a4, d4, theta4, lmin4, lmax4);
	Link l5(alpha5, a5, d5, theta5, lmin5, lmax5);
	Link l6(alpha6, a6, d6, theta6, lmin6, lmax6);
	robot.append(&l1);
	robot.append(&l2);
	robot.append(&l3);
	robot.append(&l4);
	robot.append(&l5);
	robot.append(&l6);
	println("number of dof is:");
	println(robot.getDOF());
	println("True l1 tran is: ");
	HTransform3D<> l1Tran = HTransform3D<>::DH(l1.alpha(), l1.a(), l1.d(), l1.theta());
	l1Tran.print();
	println("tran from l1 is: ");
	l1.getFrame()->getTransform().print();
	println("end tran of robot is: ");
	robot.getEndTransform().print();
	PieperSolver solver(robot);
	solver.init();
	std::vector<robot::math::Q> solution = solver.solve(robot.getEndTransform());
	println("number of solution is:");
	println(solution.size());
	for (int i=0;i<solution.size();i++)
		solution[i].print();
	return 0;
}



