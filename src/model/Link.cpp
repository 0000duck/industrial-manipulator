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
		_dHParam(_alpha, _a, _d, _theta)
{
	double st=sin(_theta); double ct=cos(_theta);double sa=sin(_alpha);double ca=cos(_alpha);
	double a11=ct;double a12=-st;double a13=0;double a14=_a;
	double a21=st*ca;double a22=ct*ca;double a23=-sa;double a24=-_d*sa;
	double a31=st*sa;double a32=ct*sa;double a33=ca;double a34=_d*ca;

	static robot::math::Rotation3D<double> rot(
												a11, a12, a13,
												a21, a22, a23,
												a31, a32, a33);
	static robot::math::Vector3D<double> vec(a14, a24, a34);
	static robot::math::HTransform3D<double> tran(vec, rot);

	static Frame frame(tran);
	_frame = &frame;
}
void Link::change(double offset)  //改变link增量
{
	_offset=offset;
	if (_sigma==0)				//link为转动副
	{
		double _theta1;
		_theta1 =_theta+_offset;
		if(_theta1>_lmax || _theta1<_lmin)
			std::cout<< "错误"<<std::endl;
		else
		{
			double st=sin(_theta1); double ct=cos(_theta1);double sa=sin(_alpha);double ca=cos(_alpha);
			double a11=ct;double a12=-st;double a13=0;double a14=_a;
			double a21=st*ca;double a22=ct*ca;double a23=-sa;double a24=-_d*sa;
			double a31=st*sa;double a32=ct*sa;double a33=ca;double a34=_d*ca;

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
				//static robot::math::Vector3D<double> vec(_a1, 0, _d);
				//static robot::math::Rotation3D<double> id(1,0,0,0,1,0,0,0,1);
				//static robot::math::HTransform3D<double> tran(vec,id);
				_frame->updateTransform(1,0,0,_a1,0,1,0,0,0,0,1,_d);

			}
	}
	getFrame();
}

void Link::reset()  //恢复link初始状态
{
	double st=sin(_theta); double ct=cos(_theta);double sa=sin(_alpha);double ca=cos(_alpha);
	double a11=ct;double a12=-st;double a13=0;double a14=_a;
	double a21=st*ca;double a22=ct*ca;double a23=-sa;double a24=-_d*sa;
	double a31=st*sa;double a32=ct*sa;double a33=ca;double a34=_d*ca;

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

const DHParameters& Link::getDHParams() const
{
	return _dHParam;
}

Link::~Link() {

}

} /* namespace model */
} /* namespace robot */
