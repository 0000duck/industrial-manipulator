/*
 * Tool.h
 *
 *  Created on: Nov 15, 2017
 *      Author: a1994846931931
 */

#ifndef TOOL_H_
#define TOOL_H_

# include "../math/Q.h"
# include "SerialLink.h"

using robot::math::Q;
using std::vector;
using robot::model::SerialLink;

namespace robot {
namespace model {

class Tool {
public:
	Tool();
	virtual ~Tool();
public:
	static bool toolGetTCP(vector<Q> joints, SerialLink::ptr robot, Vector3D<>& TCP);
};

} /* namespace model */
} /* namespace robot */

#endif /* TOOL_H_ */
