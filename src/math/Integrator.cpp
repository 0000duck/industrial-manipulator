/*
 * Integrator.cpp
 *
 *  Created on: Sep 22, 2017
 *      Author: a1994846931931
 */

#include "Integrator.h"

namespace robot {
namespace math {

Integrator::Integrator()
{

}

vector<double> Integrator::integrate(Interpolator<Vector3D<double> >::ptr positionIpr, vector<double> t)
{
	auto size = t.size();
	vector<double> length;
	length.reserve(size);
	length.push_back(0);
	Vector3D<double> prePosition = positionIpr->x(t[0]);
	Vector3D<double> curPosition;
	for (int i=1; i<(int)size; i++)
	{
		curPosition = positionIpr->x(t[i]);
		length.push_back((curPosition - prePosition).getLength());
	}
	return length;
}

Integrator::~Integrator() {
	// TODO Auto-generated destructor stub
}

} /* namespace math */
} /* namespace robot */
