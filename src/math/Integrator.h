/*
 * Integrator.h
 *
 *  Created on: Sep 22, 2017
 *      Author: a1994846931931
 */

#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

# include <vector>
# include "../trajectory/LineInterpolator.h"

using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace math {

class Integrator {
public:
	Integrator();
	vector<double> integrate(Interpolator<Vector3D<double> >::ptr positionIpr, vector<double> t);
	virtual ~Integrator();
};

} /* namespace math */
} /* namespace robot */

#endif /* INTEGRATOR_H_ */
