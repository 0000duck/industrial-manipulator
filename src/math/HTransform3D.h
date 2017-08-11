/*
 * HTransform3D.h
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

#ifndef HTRANSFORM3D_H_
#define HTRANSFORM3D_H_

# include "Rotation3D.h"
# include "Vector3D.h"

namespace robot {
namespace math {

template<typename T>
class HTransform3D {
public:
	HTransform3D(){}
	virtual ~HTransform3D(){}
private:
	Rotation3D<T> _rot;

};

} /* namespace math */
} /* namespace robot */

#endif /* HTRANSFORM3D_H_ */
