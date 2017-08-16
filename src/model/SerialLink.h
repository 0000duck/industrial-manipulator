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

using robot::kinematic::Frame;

namespace robot {
namespace model {

class SerialLink {
public:
	SerialLink();
	SerialLink(std::vector<Link*>);
	void append(Link*);
	Link* pop();
	int getDOF();
//	HTransform3D<double> getEndTransform()
	void print();
	virtual ~SerialLink();
private:
	std::vector<Link*> _linkList;
	Frame* _worldFrame;
};

} /* namespace model */
} /* namespace robot */

#endif /* SERIALLINK_H_ */
