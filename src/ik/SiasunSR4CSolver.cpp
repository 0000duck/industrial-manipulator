/*
 * SiasunSR4CSolver.cpp
 *
 *  Created on: Aug 28, 2017
 *      Author: a1994846931931
 */

#include "SiasunSR4CSolver.h"

namespace robot {
namespace ik {

SiasunSR4CSolver::SiasunSR4CSolver(robot::model::SerialLink& serialRobot) {
	_dHTable = serialRobot.getDHTable();
    _alpha1 = _dHTable[0].alpha();
    _a1 = _dHTable[0].a();
    _calpha1 = cos(_dHTable[0].alpha());
    _salpha1 = sin(_dHTable[0].alpha());
    _d1 = _dHTable[0].d();

    _alpha2 = _dHTable[1].alpha();
    _a2 = _dHTable[1].a();
    _calpha2 = cos(_dHTable[1].alpha());
    _salpha2 = sin(_dHTable[1].alpha());
    _d2 = _dHTable[1].d();

    _alpha3 = _dHTable[2].alpha();
    _a3 = _dHTable[2].a();
    _calpha3 = cos(_dHTable[2].alpha());
    _salpha3 = sin(_dHTable[2].alpha());
    _d3 = _dHTable[2].d();

    _alpha4 = _dHTable[3].alpha();
    _a4 = _dHTable[3].a();
    _calpha4 = cos(_dHTable[3].alpha());
    _salpha4 = sin(_dHTable[3].alpha());
    _d4 = _dHTable[3].d();

    _alpha5 = _dHTable[4].alpha();
    _a5 = _dHTable[4].a();
    _calpha5 = cos(_dHTable[4].alpha());
    _salpha5 = sin(_dHTable[4].alpha());
    _d5 = _dHTable[4].d();

    _alpha6 = _dHTable[5].alpha();
    _a6 = _dHTable[5].a();
    _calpha6 = cos(_dHTable[5].alpha());
    _salpha6 = sin(_dHTable[5].alpha());
    _d6 = _dHTable[5].d();

    _0Tbase = (HTransform3D<>::DH(_alpha1, _a1, _d1, 0)).inverse();
    _endTjoint6 = (HTransform3D<>::DH(0, 0, _d6, 0)).inverse();
}


std::vector<Q> SiasunSR4CSolver::solve(const HTransform3D<>& baseTend) const
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
    	// theta1
    	double theta1 = atan2(sgn*y, sgn*x);
    	double R = (pow((sgn*r -_a2), 2) + pow(z, 2) - pow(_a3, 2) - pow(_a4, 2) - pow(_d4, 2))/(2.0*_a3);
    	double a = -R - _a4;
    	double b = -2*_d4;
    	double c = _a4 - R;
    	double d = b*b - 4*a*c;
    	// theta3
    	std::vector<double> theta3s;
    	if (fabs(d) < 1e-12)
    		theta3s.push_back(atan(-b/(2*a))*2);
    	else if (d > 0)
    	{
    		theta3s.push_back(atan((-b + sqrt(d))/(2*a))*2);
    		theta3s.push_back(atan((-b - sqrt(d))/(2*a))*2);
    	}
    	else
    	{
    		// TODO
    	}
    	// theta2
    	for (std::vector<double>::iterator it2=theta3s.begin(); it2<theta3s.end(); it2++)
    	{
    		double theta3 = *it2;
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

    		solveTheta456(theta1, theta2, theta3, T06, result);
    	}
    }
    for (std::vector<Q>::iterator it = result.begin(); it != result.end(); ++it) {
        for (size_t i = 0; i<(size_t)(*it).size(); i++)
            (*it)(i) -= _dHTable[i].theta();
    }
    return result;
}

void SiasunSR4CSolver::solveTheta456(
    double theta1,
    double theta2,
    double theta3,
    HTransform3D<>& T06,
    std::vector<Q>& result) const
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
        if (_alpha6 < 0) { //Case for Z(-Y)Z rotation
            theta4 = atan2(-r23/s5, -r13/s5);
            theta6 = atan2(-r32/s5, r31/s5);
        } else { //Case for Z(-Y)Z rotation
            theta4 = atan2(r23/s5, r13/s5);
            theta6 = atan2(r32/s5, -r31/s5);
        }
    }

    q(3) = theta4;
    q(4) = theta5;
    q(5) = theta6;
    result.push_back(q);

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

bool SiasunSR4CSolver::isValid() const
{
	// TODO
	return true;
}

SiasunSR4CSolver::~SiasunSR4CSolver() {

}

} /* namespace ik */
} /* namespace robot */
