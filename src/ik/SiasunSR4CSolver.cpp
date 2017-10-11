/*
 * SiasunSR4CSolver.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: a1994846931931
 */

#include "SiasunSR4CSolver.h"
# include "../common/common.h"

using namespace robot::model;

namespace robot {
namespace ik {

SiasunSR4CSolver::SiasunSR4CSolver(robot::model::SerialLink::ptr serialRobot) {
	_serialLink = serialRobot;
	_dHTable = serialRobot->getDHTable();
    _alpha1 = _dHTable[0].alpha();
    _a1 = _dHTable[0].a();
    _calpha1 = _serialLink->getLink(0)->ca();
    _salpha1 = _serialLink->getLink(0)->sa();
    _d1 = _dHTable[0].d();

    _alpha2 = _dHTable[1].alpha();
    _a2 = _dHTable[1].a();
    _calpha2 = _serialLink->getLink(1)->ca();
    _salpha2 = _serialLink->getLink(1)->sa();
    _d2 = _dHTable[1].d();

    _alpha3 = _dHTable[2].alpha();
    _a3 = _dHTable[2].a();
    _calpha3 = _serialLink->getLink(2)->ca();
    _salpha3 = _serialLink->getLink(2)->sa();
    _d3 = _dHTable[2].d();

    _alpha4 = _dHTable[3].alpha();
    _a4 = _dHTable[3].a();
    _calpha4 = _serialLink->getLink(3)->ca();
    _salpha4 = _serialLink->getLink(3)->sa();
    _d4 = _dHTable[3].d();

    _alpha5 = _dHTable[4].alpha();
    _a5 = _dHTable[4].a();
    _calpha5 = _serialLink->getLink(4)->ca();
    _salpha5 = _serialLink->getLink(4)->sa();
    _d5 = _dHTable[4].d();

    _alpha6 = _dHTable[5].alpha();
    _a6 = _dHTable[5].a();
    _calpha6 = _serialLink->getLink(5)->ca();
    _salpha6 = _serialLink->getLink(5)->sa();
    _d6 = _dHTable[5].d();

    _0Tbase = (HTransform3D<>::DH(_alpha1, _a1, _d1, 0)).inverse();
    _endTjoint6 = ((HTransform3D<>::DH(0, 0, _d6, 0))*serialRobot->getTool()->getTransform()).inverse();
}

void SiasunSR4CSolver::init()
{
	_dHTable = _serialLink->getDHTable();
    _alpha1 = _dHTable[0].alpha();
    _a1 = _dHTable[0].a();
    _calpha1 = _serialLink->getLink(0)->ca();
    _salpha1 = _serialLink->getLink(0)->sa();
    _d1 = _dHTable[0].d();

    _alpha2 = _dHTable[1].alpha();
    _a2 = _dHTable[1].a();
    _calpha2 = _serialLink->getLink(1)->ca();
    _salpha2 = _serialLink->getLink(1)->sa();
    _d2 = _dHTable[1].d();

    _alpha3 = _dHTable[2].alpha();
    _a3 = _dHTable[2].a();
    _calpha3 = _serialLink->getLink(2)->ca();
    _salpha3 = _serialLink->getLink(2)->sa();
    _d3 = _dHTable[2].d();

    _alpha4 = _dHTable[3].alpha();
    _a4 = _dHTable[3].a();
    _calpha4 = _serialLink->getLink(3)->ca();
    _salpha4 = _serialLink->getLink(3)->sa();
    _d4 = _dHTable[3].d();

    _alpha5 = _dHTable[4].alpha();
    _a5 = _dHTable[4].a();
    _calpha5 = _serialLink->getLink(4)->ca();
    _salpha5 = _serialLink->getLink(4)->sa();
    _d5 = _dHTable[4].d();

    _alpha6 = _dHTable[5].alpha();
    _a6 = _dHTable[5].a();
    _calpha6 = _serialLink->getLink(5)->ca();
    _salpha6 = _serialLink->getLink(5)->sa();
    _d6 = _dHTable[5].d();

    _0Tbase = (HTransform3D<>::DH(_alpha1, _a1, _d1, 0)).inverse();
    _endTjoint6 = ((HTransform3D<>::DH(0, 0, _d6, 0))*_serialLink->getTool()->getTransform()).inverse();
}

std::vector<Q> SiasunSR4CSolver::solve(const HTransform3D<>& baseTend, const model::Config& config) const
{
	HTransform3D<> T06 = _0Tbase*baseTend*_endTjoint6;
    double x = T06.getPosition()(0);
    double y = T06.getPosition()(1);
    double z = T06.getPosition()(2);
    double r = sqrt(x*x + y*y);

	std::vector<Q> result;

    std::vector<double> sgns;
    sgns.push_back(1);
    sgns.push_back(-1);

    for (std::vector<double>::iterator it=sgns.begin(); it<sgns.end(); it++)
    {
    	double sgn = *it;
    	if (!isShoulderValid(sgn, config))
    	{
    	    continue;
    	}
    	// theta1 -180~180的唯一解
    	double theta1 = atan2(sgn*y, sgn*x);
    	double R = (pow((sgn*r -_a2), 2) + pow(z, 2) - pow(_a3, 2) - pow(_a4, 2) - pow(_d4, 2))/(2.0*_a3);
    	double a = -R - _a4;
    	double b = -2*_d4;
    	double c = _a4 - R;
    	double d = b*b - 4*a*c;
    	// theta3
    	std::vector<double> theta3s;
    	if (fabs(a) < 1e-12)
    		theta3s.push_back(atan((_a4 - R)/(2*a))*2);
    	else if (fabs(d) < 1e-12)
    		theta3s.push_back(atan(-b/(2*a))*2);
    	else if (d > 0)
    	{
    		theta3s.push_back(atan((-b + sqrt(d))/(2*a))*2);
    		theta3s.push_back(atan((-b - sqrt(d))/(2*a))*2);
    	}
    	else
    	{
    		cout << "theta3 no result\n";
    		cout << "d=" << d << "\n";
    		cout << "r = " << r << "\tz = " << z << "\ta2 = " << _a2 << "\ta3 = " << _a3 << "\td4 = " << _d4 << '\n';
    		cout << "x = " << x << "\ty = " << y << '\n';
    		baseTend.print();
    	}
    	// theta2
    	for (std::vector<double>::iterator it2=theta3s.begin(); it2<theta3s.end(); it2++)
    	{
    		double theta3 = *it2;
    		if (!isElbowValid(theta3, config))
    		{
				continue;
			}
    		double c3 = cos(theta3);
    		double s3 = sin(theta3);

    		double x1 = _a4*c3 - _d4*s3 + _a3;
    		double x2 = -_a4*s3 - _d4*c3;
    		double x3 = sgn*r - _a2;
    		double x4 = -_a4*s3 - _d4*c3;
    		double x5 = -_a4*c3 + _d4*s3 - _a3;
    		double x6 = z;

    		double as2 = (x3*x4 - x1*x6)/(x2*x4 - x1*x5);
    		double ac2 = (x3*x5 - x2*x6)/(x1*x5 - x2*x4);
    		double theta2 = atan2(as2, ac2);

    		solveTheta456(theta1, theta2, theta3, T06, result, config);
    	}
    }

    for (std::vector<Q>::iterator it = result.begin(); it != result.end(); ++it) {
        for (size_t i = 0; i<(size_t)(*it).size(); i++)
            (*it)(i) -= _dHTable[i].theta();
    }
    if ((int)result.size() <= 0)
    {
    	throw (std::string("错误<SiasunSR4CSolver>: 没有解!"));
    }
    /**> 根据关节范围去除不符合的结果 */
    std::vector<Q> rangeResult;
    if ((int)result.size() > 0)
    {
//		for (int i=0;;)
//		{
//			if (i >= (int)result.size())
//				break;
//			if (_serialLink->isJointValid(result[i]))
//				i++;
//			else
//				result.erase(result.begin() + i);
//		}
//	    if ((int)result.size() == 0)
//	    	throw(std::string("错误<SiasunSR4CSolver>: 根据关节范围筛选后没有解!"));
    	for (int i=0; i<(int)result.size(); i++)
    	{
    		std::vector<Q> temp = _serialLink->fixJoint(result[i]);
    		for (int k=0; k<(int)temp.size(); k++)
    		{
    			rangeResult.push_back(temp[k]);
    		}
    	}
    }
    if ((int)rangeResult.empty())
    	throw (std::string("错误<SiasunSR4CSolver>: 没有符合范围的解!"));
    return rangeResult;
}

void SiasunSR4CSolver::solveTheta456(
    double theta1,
    double theta2,
    double theta3,
    HTransform3D<>& T06,
    std::vector<Q>& result,
    const model::Config& config) const
{
	// 当前推导仅适用于末端旋转=等同于欧拉角Z（-Y）Z的情况， 其它情况可以利用欧拉角表格重新推导
    Q q(Q::zero(6));
    q(0) = theta1;
    q(1) = theta2;
    q(2) = theta3;

    HTransform3D<> T01 = HTransform3D<>::DH(0, 0, 0, theta1);
    HTransform3D<> T12 = HTransform3D<>::DH(_alpha2, _a2, _d2, theta2);
    HTransform3D<> T23 = HTransform3D<>::DH(_alpha3, _a3, _d3, theta3);
    HTransform3D<> T34 = HTransform3D<>::DH(_alpha4, _a4, _d4, 0);

    HTransform3D<> T04 = T01*T12*T23*T34;

    HTransform3D<> T46 = T04.inverse()*T06;

    double r11 = T46(0,0);
    double r12 = T46(0,1);
    double r13 = T46(0,2);

    //  double r21 = T46(1,0);
    //  double r22 = T46(1,1);
    double r23 = T46(1,2);

    double r31 = T46(2,0);
    double r32 = T46(2,1);
    double r33 = T46(2,2);

    double theta4, theta5, theta6;

    theta5 = atan2(sqrt(r31*r31+r32*r32), r33); // sin(theta5) >= 0; Case for Z(-Y)Z rotation
    if (fabs(theta5) < 1e-12) {
        theta4 = 0;
        theta6 = atan2(-r12, r11);
    }
    else if (fabs(M_PI-theta5)< 1e-12) {
        theta4 = 0;
        theta6 = atan2(r12,-r11);
    } else {
        double s5 = sin(theta5);
        //if (_alpha6 < 0)
        { //Case for Z(-Y)Z rotation
            theta4 = atan2(-r23/s5, -r13/s5);
            theta6 = atan2(-r32/s5, r31/s5);
        }
    }

    if (isWristValid(theta5, config))
    {
        q(3) = theta4;
        q(4) = theta5;
        q(5) = theta6;
    	result.push_back(q);
    }
    else
    {
		double alt4, alt6;
		if (theta4>0)
			alt4 = theta4-M_PI;
		else
			alt4 = theta4+M_PI;

		if (theta6>0)
			alt6 = theta6-M_PI;
		else
			alt6 = theta6+M_PI;

		q(3) = alt4;
		q(4) = -theta5;
		q(5) = alt6;
		result.push_back(q);
    }
}

bool SiasunSR4CSolver::isValid() const
{
	// TODO
	return true;
}

bool SiasunSR4CSolver::isShoulderValid(const robot::math::Q& q, const model::Config& config) const
{
	double j2 = q[1];
	double j3 = q[2];
	double r = _a2 + _a3*cos(j2) + _a4*cos(j2 + j3) - _d4*sin(j2 + j3);
	return isShoulderValid(r, config);
}

bool SiasunSR4CSolver::isShoulderValid(const double r, const model::Config& config) const
{
	switch (config.getShoulder())
	{
	case Config::righty:
		return (r < 0);
	case Config::lefty:
		return (r >= 0);
	case Config::ssame:
	{
		robot::math::Q q = _serialLink->getQ();
		double j2 = q[1];
		double j3 = q[2];
		double config_r = _a2 + _a3*cos(j2) + _a4*cos(j2 + j3) - _d4*sin(j2 + j3);
		return ((r > 0) == (config_r > 0));
	}
	case Config::sfree:
		return true;
	default:
		return true;
	}
}

bool SiasunSR4CSolver::isElbowValid(const robot::math::Q& q, const model::Config& config) const
{
	return isElbowValid(q[2], config);
}

bool SiasunSR4CSolver::isElbowValid(const double j3, const model::Config& config) const
{
	double fixedJ3 = j3 + atan(_d4/_a4);
	fixedJ3 = common::fixAngle(fixedJ3);
	if (fixedJ3 > M_PI)
		fixedJ3 -= 2*M_PI;
	double configJ = (_serialLink->getQ())[2] + atan(_d4/_a4);
	if (configJ > M_PI)
		configJ -= 2*M_PI;
	switch (config.getElbow())
	{
	case Config::epositive:
		return (fixedJ3 >= 0);
	case Config::enegative:
		return (fixedJ3 < 0);
	case Config::esame:
		return ((fixedJ3 >= 0) == (configJ >= 0));
	case Config::efree:
		return true;
	default:
		return true;
	}
}

bool SiasunSR4CSolver::isWristValid(const robot::math::Q& q, const model::Config& config) const
{
	return isWristValid(q[4], config);
}

bool SiasunSR4CSolver::isWristValid(const double j5, const model::Config& config) const
{
	double fixedJ5 = common::fixAngle(j5);
	switch (config.getWrist())
	{
	case Config::wpositive:
		return (fixedJ5 >= 0);
	case Config::wnegative:
		return (fixedJ5 < 0);
	case Config::wsame:
		return ((fixedJ5 >= 0) == ((_serialLink->getQ())[4] >= 0));
	case Config::wfree:
		return true;
	default:
		return true;
	}
}

Config SiasunSR4CSolver::getConfig(const robot::math::Q& q) const
{
	double j2 = q[1] + _dHTable[1].theta();
	double j3 = q[2] + _dHTable[2].theta();
	double j5 = q[4] + _dHTable[4].theta();
	j3 = common::fixAngle(j3);
	j5 = common::fixAngle(j5);
	double config_r = _a2 + _a3*cos(j2) + _a4*cos(j2 + j3) - _d4*sin(j2 + j3);
	j3 += atan(_d4/_a4);
	j3 = common::fixAngle(j3);
	double wrist = (j5 >= 0)? Config::wpositive:Config::wnegative;
	double elbow = (j3 >= 0)? Config::epositive:Config::enegative;
	double shoulder =(config_r < 0)? Config::righty:Config::lefty;
	return Config(shoulder, elbow, wrist);
}

SiasunSR4CSolver::~SiasunSR4CSolver() {

}

} /* namespace ik */
} /* namespace robot */
