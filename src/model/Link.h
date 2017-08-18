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
# include "DHParameters.h"

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
	const double a() const;
	const double d() const;
	const double theta() const;
	const double alpha() const;
	const DHParameters& getDHParams() const;
	virtual ~Link();

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	bool _sigma;
	double _offset;
	double _lmin;
	double _lmax;
	Frame* _frame;
	DHParameters _dHParam;

};

} /* namespace model */
} /* namespace robot */

#endif /* LINK_H_ */
