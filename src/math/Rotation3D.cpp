/*
 * Rotation3D.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#include "Rotation3D.h"

namespace robot {
namespace math {

template<class T>
Rotation3D<T>::Rotation3D() {
	// TODO Auto-generated constructor stub
	_m[0][0] = 1;
	_m[0][1] = 0;
	_m[0][2] = 0;
	_m[1][0] = 0;
	_m[1][1] = 1;
	_m[1][2] = 0;
	_m[2][0] = 0;
	_m[2][1] = 0;
	_m[2][2] = 1;
}

template<class T>
Rotation3D<T>::~Rotation3D() {
	// TODO Auto-generated destructor stub
}

} /* namespace math */
} /* namespace robot */
