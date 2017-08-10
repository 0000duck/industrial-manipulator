/*
 * Rotation3D.h
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#ifndef ROTATION3D_H_
#define ROTATION3D_H_

namespace robot {
namespace math {

template<class T = double>
class Rotation3D {
public:
	Rotation3D();
	virtual ~Rotation3D();
private:
	T _m[3][3];
};

} /* namespace math */
} /* namespace robot */

#endif /* ROTATION3D_H_ */
