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

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
using namespace robot::common;

int main(){
	robot::kinematic::Trsf trsf1(0,1,0,M_PI,0,0);
	HTransform3D<double>& transform1 = trsf1.getTransform();
	transform1.print();
	// update matrix using HTransform3D::setRotation
	println();
	transform1.setRotation(1,2,3,4,5,6,7,8,9);
	transform1.print();
	Frame frame1(&transform1);
	// update matrix using Frame::updateTransform
	println();
	frame1.updateTransform(1,1,1,1,2,2,2,2,3,3,3,3);
	frame1.print();
	HTransform3D<double> transform2(1, 0, 0, 0, 1, 1, 0, 0, 1, 1, 1, 0);
	Frame frame2(&transform2);
	frame2.print();
	return 0;
}



