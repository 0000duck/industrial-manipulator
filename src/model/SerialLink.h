/*
 * SerialLink.h
 *
 *  Created on: Aug 15, 2017
 *      Author: a1994846931931
 */

#ifndef SERIALLINK_H_
#define SERIALLINK_H_

# include "../kinematics/Frame.h"
# include "../math/HTransform3D.h"
# include "Link.h"
# include "DHTable.h"
# include "DHParameters.h"
# include "../math/Q.h"

using robot::kinematic::Frame;

namespace robot {
namespace model {

class SerialLink {
public:
	SerialLink(Frame* tool=NULL);
	SerialLink(std::vector<Link*>, Frame* tool=NULL);
	void append(Link*);
	Link* pop();
	int getDOF();
	DHTable getDHTable();
	HTransform3D<double> getTransform(unsigned int startLink, unsigned int endLink,const robot::math::Q& q) const;
	HTransform3D<double> getEndTransform(void) const;
	HTransform3D<double> getEndTransform(robot::math::Q& q) const;
	void print();
	virtual ~SerialLink();
private:
	std::vector<Link*> _linkList;
	Frame* _worldFrame;
	Frame* _endToTool;
};

} /* namespace model */
} /* namespace robot */

#endif /* SERIALLINK_H_ */
