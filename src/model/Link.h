/*
 * Link.h
 *
 *  Created on: Aug 14, 2017
 *      Author: zrf
 */

#ifndef LINK_H_
#define LINK_H_
# include "../math/HTransform3D.h"
namespace robot {
namespace model {

class Link {
public:
	Link(double theta,double d,double a,double alpha,bool sigma,double offset,double min,double max);

	virtual ~Link();

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	bool _sigma;
	double _offset;
	double _min;
	double _max;
	//Frame _x;

};

} /* namespace model */
} /* namespace robot */

#endif /* LINK_H_ */
