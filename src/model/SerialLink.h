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
# include "../model/Jacobian.h"
# include "Jacobian.h"
# include "Config.h"
# include "../kinematics/State.h"

using robot::kinematic::Frame;

namespace robot {
namespace model {

class SerialLink {
public:
	SerialLink(Frame* tool=NULL);
	SerialLink(std::vector<Link*>, Frame* tool=NULL);
	void append(Link*);
	Link* pop();
	int getDOF() const;
	DHTable getDHTable() const;
	HTransform3D<double> getTransform(unsigned int startLink, unsigned int endLink, const robot::math::Q& q) const;
	HTransform3D<double> getEndTransform(void) const;
	HTransform3D<double> getEndTransform(const robot::math::Q& q) const;
	Jacobian getJacobian() const;
	Jacobian getJacobian(const robot::math::Q& q) const;
	const robot::math::Q getQ() const;
	void setQ(robot::math::Q);
	Config getConfig(const Q&) const;
	const robot::math::Q getEndVelocity(const robot::kinematic::State& state) const;
	void print(); // TODO
	virtual ~SerialLink();
private:
	std::vector<Link*> _linkList;
	Frame* _worldFrame;
	Frame* _endToTool;

};

} /* namespace model */
} /* namespace robot */

#endif /* SERIALLINK_H_ */
