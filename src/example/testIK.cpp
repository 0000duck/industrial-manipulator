/*
 * test.cpp
 *
 *  Created on: Aug 23, 2017
 *      Author: a1994846931931
 */

# include "../math/Rotation3D.h"
# include "../math/HTransform3D.h"
# include "../kinematics/Trsf.h"
# include "../kinematics/Frame.h"
# include <iostream>
# include <math.h>
# include <string>
# include "../common/printAdvance.h"
//# include "model/SerialLink.h"
# include "../model/Link.h"
# include "test.h"
# include "../ik/PieperSolver.h"
# include <stdlib.h>
# include <time.h>
# include "../common/common.h"

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
using namespace robot::common;
using namespace robot::model;
using namespace robot::ik;

void ikTest()
{
	println("*** test ***");
	SerialLink robot;

//	double alpha1 = M_PI/2;
//	double alpha2 = 0;
//	double alpha3 = M_PI/2;
//	double alpha4 = -M_PI/2;
//	double alpha5 = M_PI/2;
//	double alpha6 = 0;
//	double a1 = 40;
//	double a2 = 315;
//	double a3 = 70;
//	double a4 = 0;
//	double a5 = 0;
//	double a6 = 0;
//	double d1 = 330;
//	double d2 = 0;
//	double d3 = 310;
//	double d4 = 0;
//	double d5 = 0;
////	double d6 = 70;
//	double d6 = 0;
//	double theta1 = 0;
//	double theta2 = M_PI/2;
//	double theta3 = 0;
//	double theta4 = 0;
//	double theta5 = -M_PI;
//	double theta6 = M_PI/2;

	// ***
	double alpha1 = 0;
	double alpha2 = -M_PI/2;
	double alpha3 = 0;
	double alpha4 = M_PI/2;
	double alpha5 = M_PI/2;
	double alpha6 = -M_PI/2;
	double a1 = 0;
	double a2 = 0;
	double a3 = 0.3;
	double a4 = 0;
	double a5 = 0;
	double a6 = 0;
	double d1 = 0.1;
	double d2 = 0;
	double d3 = 0;
	double d4 = 0.2;
	double d5 = 0;
	double d6 = 0;
	double theta1 = M_PI/2;
	double theta2 = -M_PI/2;
	double theta3 = M_PI/2;
	double theta4 = M_PI/2;
	double theta5 = 0;
	double theta6 = 0;
	// ***
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

	PieperSolver solver(robot);
	int MAXSTEP = 10000;
	int unsolved = 0;
	int allresult = 0;
	int correct = 0;
	int wrong = 0;
	clock_t start = clock();

	HTransform3D<> endTran;
	std::vector<Q> result;
	Jacobian J;
	for (int i=0; i<MAXSTEP; i++)
	{
		robot::math::Q q(fRand(-3.14, 3.14), fRand(-3.14, 3.14), fRand(-3.14, 3.14), fRand(-3.14, 3.14), fRand(-3.14, 3.14), fRand(-3.14, 3.14));
		endTran = robot.getEndTransform(q);
		result = solver.solve(endTran);
		if (result.size() == 0)
		{
			unsolved++;
		}
		else
		{
			allresult += result.size();
			for (unsigned int i=0; i<result.size(); i++)
			{
				if (endTran == robot.getEndTransform(result[i]))
				{
					correct++;
				}
				else
				{
					wrong++;
					println("计算错误:\n设定Q与计算Q");
					q.print();
					result[i].print();
					println("正确变换矩阵与求得的变换矩阵");
					endTran.print();
					robot.getEndTransform(result[i]).print();
				}
			}
			J = robot.getJacobian(result[0]);
			J.doInverse();
		}
	}
	clock_t end = clock();
	println("unsolved number: ");
	println(unsolved);
	println("total result: ");
	println(allresult);
	println("correct answer: ");
	println(correct);
	println("wrong answer: ");
	println(wrong);
	println("time: ");
	println(end - start);
}



