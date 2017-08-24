/*
 * Jacobian.h
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

namespace robot {
namespace model {

class Jacobian {
public:
	Jacobian();
	virtual ~Jacobian();
private:
	double _j[6][6];
};

} /* namespace model */
} /* namespace robot */

#endif /* JACOBIAN_H_ */
