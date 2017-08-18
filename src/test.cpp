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
# include "model/SerialLink.h"
# include "model/Link.h"
# include "test.h"

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
using namespace robot::common;
using namespace robot::model;

int main(){
	cout<<"test";
	SerialLink robot;
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
	return 0;
}



