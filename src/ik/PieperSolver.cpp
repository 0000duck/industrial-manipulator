/*
 * PieperSolver.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

# include "PieperSolver.h"
# include <math.h>

namespace robot {
namespace ik {

double Power(double arg, int exp) {
    double res = arg;
    for (int i = 0; i<exp-1; i++)
        res *= arg;
    return res;
}

PieperSolver::PieperSolver(robot::model::SerialLink& serialRobot) {
	_dHTable = serialRobot.getDHTable();
    alpha0 = _dHTable[0].alpha();
    a0 = _dHTable[0].a();
    calpha0 = cos(_dHTable[0].alpha());
    salpha0 = sin(_dHTable[0].alpha());
    d1 = _dHTable[0].d();

//    _0Tbase = (_baseTdhRef*HTransform3D<>::DH(alpha0, a0, d1, 0)).inverse();

    alpha1 = _dHTable[1].alpha();
    a1 = _dHTable[1].a();
    calpha1 = cos(_dHTable[1].alpha());
    salpha1 = sin(_dHTable[1].alpha());
    d2 = _dHTable[1].d();

    alpha2 = _dHTable[2].alpha();
    a2 = _dHTable[2].a();
    calpha2 = cos(_dHTable[2].alpha());
    salpha2 = sin(_dHTable[2].alpha());
    d3 = _dHTable[2].d();

    alpha3 = _dHTable[3].alpha();
    a3 = _dHTable[3].a();
    calpha3 = cos(_dHTable[3].alpha());
    salpha3 = sin(_dHTable[3].alpha());
    d4 = _dHTable[3].d();

    alpha4 = _dHTable[4].alpha();
    a4 = _dHTable[4].a();
    calpha4 = cos(_dHTable[4].alpha());
    salpha4 = sin(_dHTable[4].alpha());
    d5 = _dHTable[4].d();

    alpha5 = _dHTable[5].alpha();
    a5 = _dHTable[5].a();
    calpha5 = cos(_dHTable[5].alpha());
    salpha5 = sin(_dHTable[5].alpha());
    d6 = _dHTable[5].d();

    HTransform3D<> baseT0 = HTransform3D<>::DH(alpha0, a0, d1, 0);
    _0Tbase = baseT0.inverse();
    _endTjoint6 = ((HTransform3D<>::DH(0, 0, d6, 0))*serialRobot.getTool().getTransform()).inverse();
}

void PieperSolver::init() {
    alpha0 = _dHTable[0].alpha();
    a0 = _dHTable[0].a();
    calpha0 = cos(_dHTable[0].alpha());
    salpha0 = sin(_dHTable[0].alpha());
    d1 = _dHTable[0].d();

//    _0Tbase = (_baseTdhRef*HTransform3D<>::DH(alpha0, a0, d1, 0)).inverse();

    alpha1 = _dHTable[1].alpha();
    a1 = _dHTable[1].a();
    calpha1 = cos(_dHTable[1].alpha());
    salpha1 = sin(_dHTable[1].alpha());
    d2 = _dHTable[1].d();

    alpha2 = _dHTable[2].alpha();
    a2 = _dHTable[2].a();
    calpha2 = cos(_dHTable[2].alpha());
    salpha2 = sin(_dHTable[2].alpha());
    d3 = _dHTable[2].d();

    alpha3 = _dHTable[3].alpha();
    a3 = _dHTable[3].a();
    calpha3 = cos(_dHTable[3].alpha());
    salpha3 = sin(_dHTable[3].alpha());
    d4 = _dHTable[3].d();

    alpha4 = _dHTable[4].alpha();
    a4 = _dHTable[4].a();
    calpha4 = cos(_dHTable[4].alpha());
    salpha4 = sin(_dHTable[4].alpha());
    d5 = _dHTable[4].d();

    alpha5 = _dHTable[5].alpha();
    a5 = _dHTable[5].a();
    calpha5 = cos(_dHTable[5].alpha());
    salpha5 = sin(_dHTable[5].alpha());
    d6 = _dHTable[5].d();

    HTransform3D<> baseT0 = HTransform3D<>::DH(alpha0, a0, d1, 0);
    _0Tbase = baseT0.inverse();
//    _endTjoint6 = ((HTransform3D<>::DH(0, 0, d6, 0))*serialRobot.getTool()->getTransform()).inverse();
}

std::vector<Q> PieperSolver::solve(const HTransform3D<>& baseTend) const
{
	HTransform3D<> T06 = _0Tbase*baseTend*_endTjoint6;
//	cout << "T06 is: " << std::endl;
//	T06.print();
    double r = Power(T06.getPosition().getLengh(), 2);
    double x = T06.getPosition()(0);
    double y = T06.getPosition()(1);
    double z = T06.getPosition()(2);
//    cout<<"r= "<<r<<" "<<"x= "<<x<<" y= "<<y<<" z= "<<z<<std::endl;

    std::vector<double> theta3;
    std::vector<Q> result;
    if (a1 == 0) {
//        std::cout<<"Case 1"<<std::endl;

        theta3 = solveTheta3Case1(r);

        for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
            double theta3 = *it;
//            std::cout << "solution of theta3: " << theta3 << std::endl;
            std::vector<double> theta2sol = solveTheta2Case1(z, theta3);
            typedef std::vector<double>::iterator I;
            for (I it2 = theta2sol.begin(); it2 != theta2sol.end(); ++it2)
            {
                double theta2 = *it2;
                double theta1 = solveTheta1(x, y, theta2, theta3);
                solveTheta456(theta1, theta2, theta3, T06, result);
            }
        }
    }
    else if (fabs(salpha1) < 1e-12) {
//        std::cout<<"Case 2"<<std::endl;
        theta3 = solveTheta3Case2(z);
        for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
            double theta3 = *it;
//            std::cout << "solution of theta3: " << theta3 << std::endl;
            std::vector<double> theta2sol = solveTheta2Case2(r, theta3);
            typedef std::vector<double>::iterator I;
            for (I it2 = theta2sol.begin(); it2 != theta2sol.end(); ++it2) {
                double theta2 = *it2;
                double theta1 = solveTheta1(x, y, theta2, theta3);
                solveTheta456(theta1, theta2, theta3, T06, result);
            }
        }
    }
    else {
    	//TODO
    	/*
    	 * case 3 暂不做处理
    	 */
//        std::cout<<"Case 3"<<std::endl;
//        theta3 = solveTheta3Case3(r, z);
//      //  std::cout<<"theta3 = "<<theta3.size()<<std::endl;
//        for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
//      //      std::cout<<"t3 = "<<*it<<std::endl;
//            double theta3 = *it;
//            double theta2 = solveTheta2(r, z, theta3);
//            double theta1 = solveTheta1(x, y, theta2, theta3);
//
//            solveTheta456(theta1, theta2, theta3, T06, result);
//        }
    }

    for (std::vector<Q>::iterator it = result.begin(); it != result.end(); ++it) {
        for (size_t i = 0; i<(size_t)(*it).size(); i++)
            (*it)(i) -= _dHTable[i].theta();
    }

    return result;

}


void PieperSolver::solveTheta456(
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

    HTransform3D<> T01 = HTransform3D<>::DH(alpha0, a0, d1, theta1); // TODO DH(0, 0, 0, theta1) ?
    HTransform3D<> T12 = HTransform3D<>::DH(alpha1, a1, d2, theta2);
    HTransform3D<> T23 = HTransform3D<>::DH(alpha2, a2, d3, theta3);
    HTransform3D<> T34 = HTransform3D<>::DH(alpha3, a3, d4, 0);

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
        if (alpha5 < 0) { //Case for Z(-Y)Z rotation
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

double PieperSolver::solveTheta1(double x, double y, double theta2, double theta3) const
{
    double c2 = cos(theta2);
    double s2 = sin(theta2);
    double c3 = cos(theta3);
    double s3 = sin(theta3);


    double c1= ((a1 + a2*c2 + a3*c2*c3 - a3*calpha2*s2*s3 + d3*s2*salpha2 + calpha3*d4*s2*salpha2 +
                 c3*calpha2*d4*s2*salpha3 + c2*d4*s3*salpha3)*x +
                (a2*calpha1*s2 - d2*salpha1 - (d3 + calpha3*d4)*(calpha2*salpha1 + c2*calpha1*salpha2) +
                 a3*(c3*calpha1*s2 + c2*calpha1*calpha2*s3 - s3*salpha1*salpha2) +
                 d4*(-(c2*c3*calpha1*calpha2) + calpha1*s2*s3 + c3*salpha1*salpha2)*salpha3)*y)/
        (Power(a1,2) + Power(a3,2)*Power(c2,2)*Power(c3,2) +
         Power(a3,2)*Power(c3,2)*Power(calpha1,2)*Power(s2,2) +
         Power(a2,2)*(Power(c2,2) + Power(calpha1,2)*Power(s2,2)) -
         2*Power(a3,2)*c2*c3*calpha2*s2*s3 + 2*Power(a3,2)*c2*c3*Power(calpha1,2)*calpha2*s2*s3 +
         Power(a3,2)*Power(c2,2)*Power(calpha1,2)*Power(calpha2,2)*Power(s3,2) +
         Power(a3,2)*Power(calpha2,2)*Power(s2,2)*Power(s3,2) - 2*a3*c3*calpha1*d2*s2*salpha1 -
         2*a3*c3*calpha1*calpha2*d3*s2*salpha1 - 2*a3*c3*calpha1*calpha2*calpha3*d4*s2*salpha1 -
         2*a3*c2*calpha1*calpha2*d2*s3*salpha1 - 2*a3*c2*calpha1*Power(calpha2,2)*d3*s3*salpha1 -
         2*a3*c2*calpha1*Power(calpha2,2)*calpha3*d4*s3*salpha1 + Power(d2,2)*Power(salpha1,2) +
         2*calpha2*d2*d3*Power(salpha1,2) + Power(calpha2,2)*Power(d3,2)*Power(salpha1,2) +
         2*calpha2*calpha3*d2*d4*Power(salpha1,2) +
         2*Power(calpha2,2)*calpha3*d3*d4*Power(salpha1,2) +
         Power(calpha2,2)*Power(calpha3,2)*Power(d4,2)*Power(salpha1,2) + 2*a3*c2*c3*d3*s2*salpha2 -
         2*a3*c2*c3*Power(calpha1,2)*d3*s2*salpha2 + 2*a3*c2*c3*calpha3*d4*s2*salpha2 -
         2*a3*c2*c3*Power(calpha1,2)*calpha3*d4*s2*salpha2 -
         2*a3*Power(c2,2)*Power(calpha1,2)*calpha2*d3*s3*salpha2 -
         2*a3*Power(c2,2)*Power(calpha1,2)*calpha2*calpha3*d4*s3*salpha2 -
         2*a3*calpha2*d3*Power(s2,2)*s3*salpha2 - 2*a3*calpha2*calpha3*d4*Power(s2,2)*s3*salpha2 +
         2*c2*calpha1*d2*d3*salpha1*salpha2 + 2*c2*calpha1*calpha2*Power(d3,2)*salpha1*salpha2 +
         2*c2*calpha1*calpha3*d2*d4*salpha1*salpha2 +
         4*c2*calpha1*calpha2*calpha3*d3*d4*salpha1*salpha2 +
         2*c2*calpha1*calpha2*Power(calpha3,2)*Power(d4,2)*salpha1*salpha2 -
         2*Power(a3,2)*c3*calpha1*s2*s3*salpha1*salpha2 -
         2*Power(a3,2)*c2*calpha1*calpha2*Power(s3,2)*salpha1*salpha2 +
         2*a3*d2*s3*Power(salpha1,2)*salpha2 + 2*a3*calpha2*d3*s3*Power(salpha1,2)*salpha2 +
         2*a3*calpha2*calpha3*d4*s3*Power(salpha1,2)*salpha2 +
         Power(c2,2)*Power(calpha1,2)*Power(d3,2)*Power(salpha2,2) +
         2*Power(c2,2)*Power(calpha1,2)*calpha3*d3*d4*Power(salpha2,2) +
         Power(c2,2)*Power(calpha1,2)*Power(calpha3,2)*Power(d4,2)*Power(salpha2,2) +
         Power(d3,2)*Power(s2,2)*Power(salpha2,2) + 2*calpha3*d3*d4*Power(s2,2)*Power(salpha2,2) +
         Power(calpha3,2)*Power(d4,2)*Power(s2,2)*Power(salpha2,2) +
         2*a3*c2*calpha1*d3*s3*salpha1*Power(salpha2,2) +
         2*a3*c2*calpha1*calpha3*d4*s3*salpha1*Power(salpha2,2) +
         Power(a3,2)*Power(s3,2)*Power(salpha1,2)*Power(salpha2,2) -
         2*d4*(-(calpha1*(d2 + calpha2*(d3 + calpha3*d4))*(c2*c3*calpha2 - s2*s3)*salpha1) -
               ((d3 + calpha3*d4)*(c3*calpha2*(Power(c2,2)*Power(calpha1,2) + Power(s2,2)) -
                                   c2*(-1 + Power(calpha1,2))*s2*s3) -
                c3*(d2 + calpha2*(d3 + calpha3*d4))*Power(salpha1,2))*salpha2 +
               c2*c3*calpha1*(d3 + calpha3*d4)*salpha1*Power(salpha2,2) +
               a3*(Power(c2,2)*c3*(-1 + Power(calpha1,2)*Power(calpha2,2))*s3 -
                   c3*(calpha1 - calpha2)*(calpha1 + calpha2)*Power(s2,2)*s3 -
                   calpha1*s2*(c3 - s3)*(c3 + s3)*salpha1*salpha2 +
                   c3*s3*Power(salpha1,2)*Power(salpha2,2) +
                   c2*calpha2*((-1 + Power(calpha1,2))*s2*(c3 - s3)*(c3 + s3) -
                               2*c3*calpha1*s3*salpha1*salpha2)))*salpha3 +
         Power(d4,2)*(Power(c2,2)*(Power(c3,2)*Power(calpha1,2)*Power(calpha2,2) + Power(s3,2)) +
                      Power(s2,2)*(Power(c3,2)*Power(calpha2,2) + Power(calpha1,2)*Power(s3,2)) +
                      2*c3*calpha1*s2*s3*salpha1*salpha2 + Power(c3,2)*Power(salpha1,2)*Power(salpha2,2) -
                      2*c2*c3*calpha2*((-1 + Power(calpha1,2))*s2*s3 + c3*calpha1*salpha1*salpha2))*
         Power(salpha3,2) + 2*a1*(a2*c2 + a3*c2*c3 - a3*calpha2*s2*s3 + d3*s2*salpha2 +
                                  calpha3*d4*s2*salpha2 + c3*calpha2*d4*s2*salpha3 + c2*d4*s3*salpha3) +
         2*a2*(-(calpha1*(d2 + calpha2*(d3 + calpha3*d4))*s2*salpha1) -
               c2*(-1 + Power(calpha1,2))*(d3 + calpha3*d4)*s2*salpha2 +
               a3*(Power(c2,2)*c3 + c2*(-1 + Power(calpha1,2))*calpha2*s2*s3 +
                   calpha1*s2*(c3*calpha1*s2 - s3*salpha1*salpha2)) +
               d4*(-(c2*c3*(-1 + Power(calpha1,2))*calpha2*s2) + Power(c2,2)*s3 +
                   calpha1*s2*(calpha1*s2*s3 + c3*salpha1*salpha2))*salpha3));

    double s1 = (-((a2*calpha1*s2 - d2*salpha1 - (d3 + calpha3*d4)*(calpha2*salpha1 + c2*calpha1*salpha2) +
                    a3*(c3*calpha1*s2 + c2*calpha1*calpha2*s3 - s3*salpha1*salpha2) +
                    d4*(-(c2*c3*calpha1*calpha2) + calpha1*s2*s3 + c3*salpha1*salpha2)*salpha3)*x) +
                 (a1 + a2*c2 + a3*c2*c3 - a3*calpha2*s2*s3 + d3*s2*salpha2 + calpha3*d4*s2*salpha2 +
                  c3*calpha2*d4*s2*salpha3 + c2*d4*s3*salpha3)*y)/
        (Power(a1,2) + Power(a3,2)*Power(c2,2)*Power(c3,2) +
         Power(a3,2)*Power(c3,2)*Power(calpha1,2)*Power(s2,2) +
         Power(a2,2)*(Power(c2,2) + Power(calpha1,2)*Power(s2,2)) -
         2*Power(a3,2)*c2*c3*calpha2*s2*s3 + 2*Power(a3,2)*c2*c3*Power(calpha1,2)*calpha2*s2*s3 +
         Power(a3,2)*Power(c2,2)*Power(calpha1,2)*Power(calpha2,2)*Power(s3,2) +
         Power(a3,2)*Power(calpha2,2)*Power(s2,2)*Power(s3,2) - 2*a3*c3*calpha1*d2*s2*salpha1 -
         2*a3*c3*calpha1*calpha2*d3*s2*salpha1 - 2*a3*c3*calpha1*calpha2*calpha3*d4*s2*salpha1 -
         2*a3*c2*calpha1*calpha2*d2*s3*salpha1 - 2*a3*c2*calpha1*Power(calpha2,2)*d3*s3*salpha1 -
         2*a3*c2*calpha1*Power(calpha2,2)*calpha3*d4*s3*salpha1 + Power(d2,2)*Power(salpha1,2) +
         2*calpha2*d2*d3*Power(salpha1,2) + Power(calpha2,2)*Power(d3,2)*Power(salpha1,2) +
         2*calpha2*calpha3*d2*d4*Power(salpha1,2) +
         2*Power(calpha2,2)*calpha3*d3*d4*Power(salpha1,2) +
         Power(calpha2,2)*Power(calpha3,2)*Power(d4,2)*Power(salpha1,2) + 2*a3*c2*c3*d3*s2*salpha2 -
         2*a3*c2*c3*Power(calpha1,2)*d3*s2*salpha2 + 2*a3*c2*c3*calpha3*d4*s2*salpha2 -
         2*a3*c2*c3*Power(calpha1,2)*calpha3*d4*s2*salpha2 -
         2*a3*Power(c2,2)*Power(calpha1,2)*calpha2*d3*s3*salpha2 -
         2*a3*Power(c2,2)*Power(calpha1,2)*calpha2*calpha3*d4*s3*salpha2 -
         2*a3*calpha2*d3*Power(s2,2)*s3*salpha2 - 2*a3*calpha2*calpha3*d4*Power(s2,2)*s3*salpha2 +
         2*c2*calpha1*d2*d3*salpha1*salpha2 + 2*c2*calpha1*calpha2*Power(d3,2)*salpha1*salpha2 +
         2*c2*calpha1*calpha3*d2*d4*salpha1*salpha2 +
         4*c2*calpha1*calpha2*calpha3*d3*d4*salpha1*salpha2 +
         2*c2*calpha1*calpha2*Power(calpha3,2)*Power(d4,2)*salpha1*salpha2 -
         2*Power(a3,2)*c3*calpha1*s2*s3*salpha1*salpha2 -
         2*Power(a3,2)*c2*calpha1*calpha2*Power(s3,2)*salpha1*salpha2 +
         2*a3*d2*s3*Power(salpha1,2)*salpha2 + 2*a3*calpha2*d3*s3*Power(salpha1,2)*salpha2 +
         2*a3*calpha2*calpha3*d4*s3*Power(salpha1,2)*salpha2 +
         Power(c2,2)*Power(calpha1,2)*Power(d3,2)*Power(salpha2,2) +
         2*Power(c2,2)*Power(calpha1,2)*calpha3*d3*d4*Power(salpha2,2) +
         Power(c2,2)*Power(calpha1,2)*Power(calpha3,2)*Power(d4,2)*Power(salpha2,2) +
         Power(d3,2)*Power(s2,2)*Power(salpha2,2) + 2*calpha3*d3*d4*Power(s2,2)*Power(salpha2,2) +
         Power(calpha3,2)*Power(d4,2)*Power(s2,2)*Power(salpha2,2) +
         2*a3*c2*calpha1*d3*s3*salpha1*Power(salpha2,2) +
         2*a3*c2*calpha1*calpha3*d4*s3*salpha1*Power(salpha2,2) +
         Power(a3,2)*Power(s3,2)*Power(salpha1,2)*Power(salpha2,2) -
         2*d4*(-(calpha1*(d2 + calpha2*(d3 + calpha3*d4))*(c2*c3*calpha2 - s2*s3)*salpha1) -
               ((d3 + calpha3*d4)*(c3*calpha2*(Power(c2,2)*Power(calpha1,2) + Power(s2,2)) -
                                   c2*(-1 + Power(calpha1,2))*s2*s3) -
                c3*(d2 + calpha2*(d3 + calpha3*d4))*Power(salpha1,2))*salpha2 +
               c2*c3*calpha1*(d3 + calpha3*d4)*salpha1*Power(salpha2,2) +
               a3*(Power(c2,2)*c3*(-1 + Power(calpha1,2)*Power(calpha2,2))*s3 -
                   c3*(calpha1 - calpha2)*(calpha1 + calpha2)*Power(s2,2)*s3 -
                   calpha1*s2*(c3 - s3)*(c3 + s3)*salpha1*salpha2 +
                   c3*s3*Power(salpha1,2)*Power(salpha2,2) +
                   c2*calpha2*((-1 + Power(calpha1,2))*s2*(c3 - s3)*(c3 + s3) -
                               2*c3*calpha1*s3*salpha1*salpha2)))*salpha3 +
         Power(d4,2)*(Power(c2,2)*(Power(c3,2)*Power(calpha1,2)*Power(calpha2,2) + Power(s3,2)) +
                      Power(s2,2)*(Power(c3,2)*Power(calpha2,2) + Power(calpha1,2)*Power(s3,2)) +
                      2*c3*calpha1*s2*s3*salpha1*salpha2 + Power(c3,2)*Power(salpha1,2)*Power(salpha2,2) -
                      2*c2*c3*calpha2*((-1 + Power(calpha1,2))*s2*s3 + c3*calpha1*salpha1*salpha2))*
         Power(salpha3,2) + 2*a1*(a2*c2 + a3*c2*c3 - a3*calpha2*s2*s3 + d3*s2*salpha2 +
                                  calpha3*d4*s2*salpha2 + c3*calpha2*d4*s2*salpha3 + c2*d4*s3*salpha3) +
         2*a2*(-(calpha1*(d2 + calpha2*(d3 + calpha3*d4))*s2*salpha1) -
               c2*(-1 + Power(calpha1,2))*(d3 + calpha3*d4)*s2*salpha2 +
               a3*(Power(c2,2)*c3 + c2*(-1 + Power(calpha1,2))*calpha2*s2*s3 +
                   calpha1*s2*(c3*calpha1*s2 - s3*salpha1*salpha2)) +
               d4*(-(c2*c3*(-1 + Power(calpha1,2))*calpha2*s2) + Power(c2,2)*s3 +
                   calpha1*s2*(calpha1*s2*s3 + c3*salpha1*salpha2))*salpha3));

    return atan2(s1, c1);
}


std::vector<double> PieperSolver::solveTheta2Case2(double r, double theta3) const {
    double c3 = cos(theta3);
    double s3 = sin(theta3);

    double a = -Power(a1,2) + 2*a1*a2 - Power(a2,2) - Power(a3,2) + 2*a1*a3*c3 - 2*a2*a3*c3 - Power(d2,2) -
        2*calpha2*d2*d3 - Power(d3,2) - 2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r -
        2*a3*d2*s3*salpha2 + 2*a1*d4*s3*salpha3 - 2*a2*d4*s3*salpha3 + 2*c3*d2*d4*salpha2*salpha3;

    double b = 4*a1*a3*calpha2*s3 - 4*a1*d3*salpha2 - 4*a1*calpha3*d4*salpha2 - 4*a1*c3*calpha2*d4*salpha3;

    double c = -Power(a1,2) - 2*a1*a2 - Power(a2,2) - Power(a3,2) - 2*a1*a3*c3 - 2*a2*a3*c3 - Power(d2,2) -
        2*calpha2*d2*d3 - Power(d3,2) - 2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r -
        2*a3*d2*s3*salpha2 - 2*a1*d4*s3*salpha3 - 2*a2*d4*s3*salpha3 + 2*c3*d2*d4*salpha2*salpha3;


    double d = b*b-4*a*c;
    std::vector<double> result;
    if (fabs(a)<1e-12) {
        result.push_back(-c/b);
    }
    else if (fabs(d) < 1e-12)
        result.push_back(-b/(2*a));
    else if (d>0) {
        result.push_back((-b-sqrt(d))/(2*a));
        result.push_back((-b+sqrt(d))/(2*a));
    }


    for (std::vector<double>::iterator it = result.begin(); it != result.end(); ++it) {
        double u = *it;
        double c2 = (1-u*u)/(1+u*u);
        double s2 = 2*u/(1+u*u);
        double theta2 = atan2(s2,c2);
        (*it) = theta2;
//        std::cout << "solution of theta2: " << theta2 << std::endl;
    }

    return result;

}

std::vector<double> PieperSolver::solveTheta2Case1(double z, double theta3) const {
    double c3 = cos(theta3);
    double s3 = sin(theta3);

    /*double a = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 + a3*calpha2*s3*salpha1 -
        a3*calpha1*s3*salpha2 - d3*salpha1*salpha2 - calpha3*d4*salpha1*salpha2 -
        c3*calpha2*d4*salpha1*salpha3 + c3*calpha1*d4*salpha2*salpha3 + z;

    double b = -2*a2*salpha1 - 2*a3*c3*salpha1 - 2*d4*s3*salpha1*salpha3;

    double c = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 - a3*calpha2*s3*salpha1 -
        a3*calpha1*s3*salpha2 + d3*salpha1*salpha2 + calpha3*d4*salpha1*salpha2 +
        c3*calpha2*d4*salpha1*salpha3 + c3*calpha1*d4*salpha2*salpha3 + z;
	*/
    double f1 = a3*c3 + d4*salpha3*s3 + a2;
    double f2 = a3*calpha2*s3 - d4*salpha3*calpha2*c3 - d4*salpha2*calpha3 - d3*salpha2;
    double f3 = a3*salpha2*s3 - d4*salpha3*salpha2*c3 + d4*calpha2*calpha3 + d3*calpha2;

    double k1 = f1;
    double k2 = -f2;
    double k4 = f3*calpha1 + d2*calpha1;

    double a = k4 - z + k2*salpha1;
    double b = 2*k1*salpha1;
    double c = k4 - z - k2*salpha1;

    double d = b*b-4*a*c;

    std::vector<double> result;
    if (fabs(a)<1e-12)
        result.push_back(-c/b);
    else if (fabs(d) < 1e-12)
        result.push_back(-b/(2*a));
    else if (d>0) {
        result.push_back((-b-sqrt(d))/(2.0*a));
        result.push_back((-b+sqrt(d))/(2.0*a));
    }


    for (std::vector<double>::iterator it = result.begin(); it != result.end(); ++it) {
        double u = *it;
        double c2 = (1-u*u)/(1+u*u);
        double s2 = 2*u/(1+u*u);
        double theta2 = atan2(s2,c2);
        (*it) = theta2;
//        std::cout << "solution of theta2: " << theta2 << std::endl;
    }

    return result;
}

std::vector<double> PieperSolver::solveTheta3Case1(double r) const {
    double a = -Power(a1,2) - Power(a2,2) + 2*a2*a3 - Power(a3,2) - Power(d2,2) - 2*calpha2*d2*d3 - Power(d3,2) -
        2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r - 2*d2*d4*salpha2*salpha3;

    double b = -4*a3*d2*salpha2 - 4*a2*d4*salpha3;

    double c = -Power(a1,2) - Power(a2,2) - 2*a2*a3 - Power(a3,2) - Power(d2,2) - 2*calpha2*d2*d3 - Power(d3,2)
        -2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r + 2*d2*d4*salpha2*salpha3;

    double d = b*b-4*a*c;
//    cout << a <<" "<< b <<" "<<c<<" "<<d<<std::endl;
    std::vector<double> result;
    if (fabs(a) < 1e-12)  //Is it only a linear equation
        result.push_back(-c/b);
    else if (fabs(d) < 1e-12)
        result.push_back(-b/(2*a));
    else if (d>0) {
        result.push_back((-b-sqrt(d))/(2*a));
        result.push_back((-b+sqrt(d))/(2*a));
    }


    for (std::vector<double>::iterator it = result.begin(); it != result.end(); ++it) {
        double u = *it;
        double c3 = (1-u*u)/(1+u*u);
        double s3 = 2*u/(1+u*u);
        double theta3 = atan2(s3,c3);
        (*it) = theta3;
    }

    return result;
}


std::vector<double> PieperSolver::solveTheta3Case2(double z) const {
    double a = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 - calpha1*d4*salpha2*salpha3 + z;

    double b = -2*a3*calpha1*salpha2;

    double c = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 + calpha1*d4*salpha2*salpha3 + z;


    double d = b*b-4*a*c;
    std::vector<double> result;
    if (fabs(d) < 1e-12)
        result.push_back(-b/(2*a));
    else if (d>0) {
        result.push_back((-b-sqrt(d))/(2*a));
        result.push_back((-b+sqrt(d))/(2*a));
    }

    for (std::vector<double>::iterator it = result.begin(); it != result.end(); ++it) {
        double u = *it;
        double c3 = (1-u*u)/(1+u*u);
        double s3 = 2*u/(1+u*u);
        double theta3 = atan2(s3,c3);
        (*it) = theta3;
    }

    return result;

}

bool PieperSolver::isValid() const
{
	if (a4==0&&a5==0&&d4==0&&(a1==0||fabs(sin(alpha1))<1e-12))
		return true;
	return false;
}

PieperSolver::~PieperSolver() {
}

} /* namespace ik */
} /* namespace robot */
