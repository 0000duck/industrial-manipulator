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
# include "../kinematics/Frame.h"
# include "DHParameters.h"

using namespace robot::kinematic;

namespace robot {
namespace model {

Frame* Link::getFrame()
{
	 return _frame;
}

Link::Link(double alpha, double a, double d, double theta, double min, double max, bool sigma):
		_theta(theta),_d(d),_a(a),_alpha(alpha),
		_sigma(sigma),_offset(0),_lmin(min),_lmax(max),
		_dHParam(_alpha, _a, _d, _theta),
		_sa(sin(_alpha)), _ca(cos(_alpha)),
		_st(sin(theta)), _ct(cos(theta))
{
	double a11=_ct;double a12=-_st;double a13=0;double a14=_a;
	double a21=_st*_ca;double a22=_ct*_ca;double a23=-_sa;double a24=-_d*_sa;
	double a31=_st*_sa;double a32=_ct*_sa;double a33=_ca;double a34=_d*_ca;

	robot::math::Rotation3D<double> rot(
												a11, a12, a13,
												a21, a22, a23,
												a31, a32, a33);
	robot::math::Vector3D<double> vec(a14, a24, a34);
	robot::math::HTransform3D<double> tran(vec, rot);

	_frame = new Frame(tran);
}
void Link::change(double offset)  //改变link增量
{
	_offset=offset;
	if (_sigma==0)				//link为转动副
	{
		double theta1;
		theta1 =_theta+_offset;
		if(theta1>_lmax || theta1<_lmin)
			std::cout<< "错误"<<std::endl;
		else
		{
			double _st=sin(theta1); double _ct=cos(theta1);
			double a11=_ct;double a12=-_st;double a13=0;double a14=_a;
			double a21=_st*_ca;double a22=_ct*_ca;double a23=-_sa;double a24=-_d*_sa;
			double a31=_st*_sa;double a32=_ct*_sa;double a33=_ca;double a34=_d*_ca;

			_frame->updateTransform(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);

		}
	}
	else if(_sigma==1)				//link为移动副
	{
		double _a1=_a+offset;
		if(_a1>_lmax || _a1<_lmin)
			std::cout<< "错误"<<std::endl;
			else
			{
				_frame->updateTransform(1,0,0,_a1,0,1,0,0,0,0,1,_d);
			}
	}
	getFrame();
}

void Link::reset()  //恢复link初始状态
{
	double _st=sin(_theta); double _ct=cos(_theta);
	double a11=_ct;double a12=-_st;double a13=0;double a14=_a;
	double a21=_st*_ca;double a22=_ct*_ca;double a23=-_sa;double a24=-_d*_sa;
	double a31=_st*_sa;double a32=_ct*_sa;double a33=_ca;double a34=_d*_ca;

	_frame->updateTransform(a11, a12, a13, a14, a21, a22, a23, a24, a31, a32, a33, a34);

	getFrame();
}

const double Link::a() const
{
	return _a;
}

const double Link::d() const
{
	return _d;
}

const double Link::theta() const
{
	return _theta;
}

const double Link::alpha() const
{
	return _alpha;
}

double Link::lmin() const
{
	return _lmin;
}


double Link::lmax() const
{
	return _lmax;
}

const DHParameters& Link::getDHParams() const
{
	return _dHParam;
}

HTransform3D<> Link::getTransform(double q) const
{
	return HTransform3D<>::DH(_alpha, _a, _d, _theta + q);
}

Link::~Link()
{
	delete _frame;
}

} /* namespace model */
} /* namespace robot */
