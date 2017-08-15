/*
 * Link.cpp
 *
 *  Created on: Aug 14, 2017
 *      Author: zrf
 */

#include "Link.h"
#include <iostream>
# include "../math/HTransform3D.h"
# include <math.h>

namespace robot {
namespace model {

Link::Link(double theta,double d,double a,double alpha,bool sigma,double offset,double min,double max)
{
	// TODO Auto-generated constructor stub
	_theta=theta;
	_d=d;
	_a=a;
	_alpha=alpha;
	_sigma=sigma;
	_offset=offset;
	_min=min;
	_max=max;

	if (_sigma==0)
	{
		_theta =_theta+_offset;
		if(_theta>_max || _theta<_min)
			std::cout<< "错误"<<std::endl;
		else
		{
			double a11=cos(_theta);double a12=-sin(_theta);double a13=0;double a14=_a;
			double a21=sin(_theta)*cos(_alpha);double a22=cos(_theta)*cos(_alpha);double a23=-sin(_alpha);double a24=-_d*sin(_alpha);
			double a31=sin(_theta)*sin(_alpha);double a32=cos(_theta)*sin(_alpha);double a33=cos(_alpha);double a34=_d*cos(_alpha);


			static robot::math::Rotation3D<double> rot(
														a11, a12, a13,
														a21, a22, a23,
														a31, a32, a33);
			static robot::math::Vector3D<double> vec(a14, a24, a34);
			static robot::math::HTransform3D<double> tran(vec, rot);
		}
	}
}

Link::~Link() {
	// TODO Auto-generated destructor stub
}

} /* namespace model */
} /* namespace robot */
