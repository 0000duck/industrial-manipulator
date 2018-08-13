/*
 * Tool.cpp
 *
 *  Created on: Nov 15, 2017
 *      Author: a1994846931931
 */

#include "Tool.h"
# include "../math/LeastSquare.h"

namespace robot {
namespace model {

Tool::Tool() {
	// TODO Auto-generated constructor stub

}

Tool::~Tool() {
	// TODO Auto-generated destructor stub
}

bool Tool::toolGetTCP(vector<Q> joints, SerialLink::ptr robot, Vector3D<>& TCP)
{
	Frame* originalTool = robot->getTool();
	HTransform3D<> T1 = robot->getEndTransform(joints[0]);
	HTransform3D<> T2 = robot->getEndTransform(joints[1]);
	HTransform3D<> T3 = robot->getEndTransform(joints[2]);
	robot->setTool(originalTool);
	vector<vector<double> > A{
		vector<double>{T1(0, 0) - T2(0, 0), T1(0, 1) - T2(0, 1), T1(0, 2) - T2(0, 2)},
		vector<double>{T1(1, 0) - T2(1, 0), T1(1, 1) - T2(1, 1), T1(1, 2) - T2(1, 2)},
		vector<double>{T1(2, 0) - T2(2, 0), T1(2, 1) - T2(2, 1), T1(2, 2) - T2(2, 2)},
		vector<double>{T2(0, 0) - T3(0, 0), T2(0, 1) - T3(0, 1), T2(0, 2) - T3(0, 2)},
		vector<double>{T2(1, 0) - T3(1, 0), T2(1, 1) - T3(1, 1), T2(1, 2) - T3(1, 2)},
		vector<double>{T2(2, 0) - T3(2, 0), T2(2, 1) - T3(2, 1), T2(2, 2) - T3(2, 2)}
	};
	vector<double> b{
		T1(0, 3) - T2(0, 3),
		T1(1, 3) - T2(1, 3),
		T1(2, 3) - T2(2, 3),
		T2(0, 3) - T3(0, 3),
		T2(1, 3) - T3(1, 3),
		T2(2, 3) - T3(2, 3)
	};
	vector<double> result;
	if (robot::math::LeastSquare::fromMatrix(A, b, result))
	{
		TCP = Vector3D<>(result[0], result[1], result[2]);
		return true;
	}
	return false;
}

} /* namespace model */
} /* namespace robot */
