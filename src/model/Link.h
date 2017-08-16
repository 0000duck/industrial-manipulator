/*
 * Link.h
 *
 *  Created on: Aug 14, 2017
 *      Author: zrf
 */

#ifndef LINK_H_
#define LINK_H_
# include "../math/HTransform3D.h"
# include "../kinematics/Frame.h"

using namespace robot::kinematic;

namespace robot {
namespace model {

class Link {
public:
	Link();
	Link(double theta,double d,double a,double alpha,bool sigma,double min,double max);
	void change(double offset);
	void reset();
	Frame* getFrame();
	virtual ~Link();

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	bool _sigma;
	double _lmin;
	double _lmax;
	double _offset;
	Frame* _frame;

};

} /* namespace model */
} /* namespace robot */

#endif /* LINK_H_ */
