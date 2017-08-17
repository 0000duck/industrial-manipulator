/*
 * PieperSolver.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#include "PieperSolver.h"
# include "../../ext/Eigen/Eigen"
# include "../../ext/rw/LinearAlgebra.hpp"

using namespace rw::math;

namespace robot {
namespace ik {

double Power(double arg, int exp) {
    double res = arg;
    for (int i = 0; i<exp-1; i++)
        res *= arg;
    return res;
}

PieperSolver::PieperSolver(robot::model::SerialLink& serialRobot) {
	// TODO Auto-generated constructor stub

}

std::vector<robot::math::Q> PieperSolver::solve(const robot::math::HTransform3D<>& T06)
{
	double r = T06.getPosition().getLengh();
	double x = T06.getPosition()(0);
	double y = T06.getPosition()(1);
	double z = T06.getPosition()(2);

	std::vector<double> theta3;
	std::vector<Q> result;
	if (a1 == 0) {
	  //  std::cout<<"Case 1"<<std::endl;

		theta3 = solveTheta3Case1(r);

		for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
			double theta3 = *it;
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
	  //  std::cout<<"Case 2"<<std::endl;
		theta3 = solveTheta3Case2(z);
		for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
			double theta3 = *it;
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
	  //  std::cout<<"Case 3"<<std::endl;
		theta3 = solveTheta3Case3(r, z);
	  //  std::cout<<"theta3 = "<<theta3.size()<<std::endl;
		for (std::vector<double>::iterator it = theta3.begin(); it != theta3.end(); ++it) {
	  //      std::cout<<"t3 = "<<*it<<std::endl;
			double theta3 = *it;
			double theta2 = solveTheta2(r, z, theta3);
			double theta1 = solveTheta1(x, y, theta2, theta3);

			solveTheta456(theta1, theta2, theta3, T06, result);
		}
	}

	for (std::vector<Q>::iterator it = result.begin(); it != result.end(); ++it) {
		for (int i = 0; i<(int)(*it).size(); i++)
			(*it)(i) -= _dHTable(i).theta();
	}

	return result;
}

void PieperSolver::solveTheta456(
    double theta1,
    double theta2,
    double theta3,
    const HTransform3D<>& T06,
    std::vector<Q>& result) const
{
    Q q(Q::zero(6));
    q(0) = theta1;
    q(1) = theta2;
    q(2) = theta3;

    HTransform3D<> T01 = HTransform3D<>::DH(alpha0, a0, d1, theta1);
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

    theta5 = atan2(sqrt(r31*r31+r32*r32), r33);
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
    }

    return result;

}

std::vector<double> PieperSolver::solveTheta2Case1(double z, double theta3) const {
    double c3 = cos(theta3);
    double s3 = sin(theta3);

    double a = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 + a3*calpha2*s3*salpha1 -
        a3*calpha1*s3*salpha2 - d3*salpha1*salpha2 - calpha3*d4*salpha1*salpha2 -
        c3*calpha2*d4*salpha1*salpha3 + c3*calpha1*d4*salpha2*salpha3 + z;

    double b = -2*a2*salpha1 - 2*a3*c3*salpha1 - 2*d4*s3*salpha1*salpha3;

    double c = -(calpha1*d2) - calpha1*calpha2*d3 - calpha1*calpha2*calpha3*d4 - a3*calpha2*s3*salpha1 -
        a3*calpha1*s3*salpha2 + d3*salpha1*salpha2 + calpha3*d4*salpha1*salpha2 +
        c3*calpha2*d4*salpha1*salpha3 + c3*calpha1*d4*salpha2*salpha3 + z;


    double d = b*b-4*a*c;

    std::vector<double> result;
    if (fabs(a)<1e-12)
        result.push_back(-c/b);
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
    }

    return result;



}

double PieperSolver::solveTheta2(double r, double z, double theta3) const {
    double c3 = cos(theta3);
    double s3 = sin(theta3);

    double c2 = -(Power(a1,2)*salpha1*(a2 + a3*c3 + d4*s3*salpha3) +
                  salpha1*(a2 + a3*c3 + d4*s3*salpha3)*
                  (Power(a2,2) + Power(a3,2) + Power(d2,2) + 2*calpha2*d2*d3 + Power(d3,2) +
                   2*calpha3*(calpha2*d2 + d3)*d4 + Power(d4,2) - r + 2*a3*d2*s3*salpha2 -
                   2*c3*d2*d4*salpha2*salpha3 + 2*a2*(a3*c3 + d4*s3*salpha3)) +
                  2*a1*(a3*calpha2*s3 - d3*salpha2 - calpha3*d4*salpha2 - c3*calpha2*d4*salpha3)*
                  (calpha1*(d2 + calpha2*d3 + calpha2*calpha3*d4 + a3*s3*salpha2 - c3*d4*salpha2*salpha3) - z)
        )/(2.*a1*salpha1*(Power(a2,2) + Power(a3,2)*(Power(c3,2) + Power(calpha2,2)*Power(s3,2)) -
                          2*a3*calpha2*(d3 + calpha3*d4)*s3*salpha2 + Power(d3 + calpha3*d4,2)*Power(salpha2,2) -
                          2*a3*c3*(-1 + Power(calpha2,2))*d4*s3*salpha3 +
                          2*c3*calpha2*d4*(d3 + calpha3*d4)*salpha2*salpha3 +
                          Power(d4,2)*(Power(c3,2)*Power(calpha2,2) + Power(s3,2))*Power(salpha3,2) +
                          2*a2*(a3*c3 + d4*s3*salpha3)));

    double s2 = (Power(a1,2)*salpha1*(a3*calpha2*s3 - d3*salpha2 - calpha3*d4*salpha2 - c3*calpha2*d4*salpha3) +
                 salpha1*(a3*calpha2*s3 - d3*salpha2 - calpha3*d4*salpha2 - c3*calpha2*d4*salpha3)*
                 (Power(a2,2) + Power(a3,2) + Power(d2,2) + 2*calpha2*d2*d3 + Power(d3,2) +
                  2*calpha3*(calpha2*d2 + d3)*d4 + Power(d4,2) - r + 2*a3*d2*s3*salpha2 -
                  2*c3*d2*d4*salpha2*salpha3 + 2*a2*(a3*c3 + d4*s3*salpha3)) -
                 2*a1*(a2 + a3*c3 + d4*s3*salpha3)*(calpha1*
                                                    (d2 + calpha2*d3 + calpha2*calpha3*d4 + a3*s3*salpha2 - c3*d4*salpha2*salpha3) - z))/
        (2.*a1*salpha1*(Power(a2,2) + Power(a3,2)*(Power(c3,2) + Power(calpha2,2)*Power(s3,2)) -
                        2*a3*calpha2*(d3 + calpha3*d4)*s3*salpha2 + Power(d3 + calpha3*d4,2)*Power(salpha2,2) -
                        2*a3*c3*(-1 + Power(calpha2,2))*d4*s3*salpha3 +
                        2*c3*calpha2*d4*(d3 + calpha3*d4)*salpha2*salpha3 +
                        Power(d4,2)*(Power(c3,2)*Power(calpha2,2) + Power(s3,2))*Power(salpha3,2) +
                        2*a2*(a3*c3 + d4*s3*salpha3)));

    return atan2(s2, c2);
}



std::vector<double> PieperSolver::solveTheta3Case1(double r) const {
    double a = -Power(a1,2) - Power(a2,2) + 2*a2*a3 - Power(a3,2) - Power(d2,2) - 2*calpha2*d2*d3 - Power(d3,2) -
        2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r - 2*d2*d4*salpha2*salpha3;

    double b = -4*a3*d2*salpha2 - 4*a2*d4*salpha3;

    double c = -Power(a1,2) - Power(a2,2) - 2*a2*a3 - Power(a3,2) - Power(d2,2) - 2*calpha2*d2*d3 - Power(d3,2)
        -2*calpha2*calpha3*d2*d4 - 2*calpha3*d3*d4 - Power(d4,2) + r + 2*d2*d4*salpha2*salpha3;

    double d = b*b-4*a*c;
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

std::vector<double> PieperSolver::solveTheta3Case3(double r, double z) const {
    //coefficients for the equation au^4+bu^3+cu^2+du+e
    setupCoefficients(r, z);
    std::vector<double> solutions;


    solutions = fSolve();

/*    std::vector<double> ddfsol = ddfSolve();
    std::vector<double> dfsol;
    switch (ddfsol.size()) {
    case 0:
        dfsol = dfSolve(0, 0);
        break;
    case 1:
        dfsol = dfSolve(ddfsol[0], ddfsol[0]);
        break;
    case 2:
        dfsol = dfSolve(ddfsol[0], ddfsol[1]);
        break;
    }


    switch (dfsol.size()) {
    case 1:
        solutions = fSolve(dfsol[0], dfsol[0], dfsol[0]);
        break;
    case 2:
        solutions = fSolve(dfsol[0], dfsol[0], dfsol[1]);
        break;
    case 3:
        solutions = fSolve(dfsol[0], dfsol[1], dfsol[2]);
        break;
    }

*/
    for (std::vector<double>::iterator it = solutions.begin(); it != solutions.end(); ++it) {
        double u = *it;
        double c3 = (1-u*u)/(1+u*u);
        double s3 = 2*u/(1+u*u);
        double theta3 = atan2(s3,c3);
        (*it) = theta3;
    }
    return solutions;

}


double PieperSolver::f(double x) const {
    return (((a*x+b)*x+c)*x+d)*x+e;
}

double PieperSolver::df(double x) const {
    return ((4*a*x+3*b)*x+2*c)*x+d;
}

double PieperSolver::ddf(double x) const {
    return (12*a*x+6*b)*x+2*c;
}

std::vector<double> PieperSolver::fSolve() const {
//    Eigen::MatrixXd A(Eigen::MatrixXd::Zero(4,4));
    Eigen::MatrixXd A(4, 4);
    A(0,0) = -b/a;
    A(0,1) = -c/a;
    A(0,2) = -d/a;
    A(0,3) = -e/a;

    A(1,0) = 1; A(1,1) = A(1,2) = A(1,3) = 0;
    A(2,1) = 1; A(2,0) = A(2,2) = A(2,3) = 0;
    A(3,2) = 1; A(3,0) = A(3,1) = A(3,3) = 0;

   // std::cout<<"a = "<<a<<" b = "<<b<<" c = "<<c<<" d = "<<d<<" e = "<<e<<std::endl;

    std::pair<Eigen::MatrixXcd, Eigen::VectorXcd> eigen = LinearAlgebra::eigenDecomposition(A);

    std::vector<double> result;
    for (size_t i = 0; i<(std::size_t)eigen.second.size(); i++) {
//        std::cout<<"Solution = "<<eigen.second(i)<<std::endl;
        if (eigen.second(i).imag() == 0)
            result.push_back(eigen.second(i).real());

    }
    return result;
}

std::vector<double> PieperSolver::fSolve(double s1, double s2, double s3) const {
    double t1 = std::min(std::min(s1,s2),s3); //Get the smallest
    double t2 = std::max(std::max(std::min(s1,s2), std::min(s1,s3)), std::min(s2,s3)); //Get the middle
    double t3 = std::max(std::max(s1,s2),s3); //Get the largest

    const double EPS = 1e-6;
    const double PREC = 1e-12;

    bool has_solution[4] = {false, false, false, false};
    double solution[4];
    int solution_count = 0;

    /* Solve to the left of t1 */
    double x = t1-EPS;
    double fval = f(x);
    double g = df(x);
    if ( (fval>0 && g>0) || (fval<0 && g<0) ) {
        while (fabs(fval)>PREC && fabs(fval/g) > PREC) {
            //std::cout<<"fval1 = "<<fval<<"  "<<g<<std::endl;
            x -= fval/g;
            fval = f(x);
            g = df(x);
        }
        has_solution[0] = true;
        solution[0] = x;
        ++solution_count;
    }
    /* Solve to the right of t3 */
    x = t3+EPS;
    fval = f(x);
    g = df(x);

    if ( (fval>0 && g <0) || (fval<0 && g>0) ) {
        while (fabs(fval)>PREC && fabs(fval/g)>PREC) {
            //std::cout<<"fval2 = "<<fval<<"  "<<g<<std::endl;
            x -= fval/g;
            fval = f(x);
            g = df(x);
        }
        has_solution[1] = true;
        solution[1] = x;
        ++solution_count;
    }

    /* Solve between t1 and t2 */
    if (t1 != t2) {
        double s1 = t1;
        double s2 = t2;
        double f1 = f(s1);
        double f2 = f(s2);
        if (f1*f2<0) { // They are on opposite sides 0
            //      g = (f2-f1)/(s2-s1);
            x = (s1+s2)/2;
            double fval = f(x);

            while (fabs(s2-s1) > PREC ) {
           //     std::cout<<"fval3 = "<<fval<<"  "<<s2-s1<<std::endl;
                //      x -= f/g;
                fval = f(x);
                if (fval*f1<0) {
                    s2 = x;
                    f2 = fval;
                }
                else {
                    s1 = x;
                    f1 = fval;
                }
                //      g = (f2-f1)/(s2-s1);
                x = (s1+s2)/2;
            }
            has_solution[2] = true;
            solution[2] = x;
            ++solution_count;
        }
    }
    else {
        if (fabs(f(t1)) < PREC) {
            has_solution[2] = true;
            solution[2] = t1;
            ++solution_count;
        }
    }

    /* Solve between t2 and t3 */
    if (t2 != t3) {
        double s1 = t2;
        double s2 = t3;
        double f1 = f(s1);
        double f2 = f(s2);
        if (f1*f2< PREC) { // They are on opposite sides of 0 or close to 0
            //      g = (f2-f1)/(s2-s1);
            x = (s1+s2)/2.0;
            double fval = f(x);
            while (fabs(s2-s1) > PREC ) {
         //       std::cout<<"fval4 = "<<fval<<"  "<<s2-s1<<std::endl;
                //      x -= f/g;
                fval = f(x);
                if (fval*f1<0) {
                    s2 = x;
                    f2 = fval;
                }
                else {
                    s1 = x;
                    f1 = fval;
                }
                //g = (f2-f1)/(s2-s1);
                x = (s1+s2)/2.0;
            }

            has_solution[3] = true;
            solution[3] = x;
            ++solution_count;
        }
    }
    else {
        if (t1 != t2 && fabs(f(t2)) < PREC) {
            has_solution[3] = true;
            solution[3] = t1;
            ++solution_count;
        }
    }

    std::vector<double> result;
    for (int i = 0; i<4; i++) {
        if (has_solution[i]) {
            result.push_back(solution[i]);
        }
    }
    return result;
}

std::vector<double> PieperSolver::dfSolve(double s1, double s2) const {
    if (s1 > s2) {
        double tmp = s1;
        s1 = s2;
        s2 = tmp;
    }

    const double EPS = 1e-6;
    const double PREC = 1e-12;

    bool has_solution[3] = {false, false, false};
    double solution[3];
    int solution_count = 0;

    /* Solve to the left of s1 */
    double x = s1-EPS;
    double fval = df(x);
    double g = ddf(x);
    if ( (fval>0 && g>0) || (fval<0 && g<0) ) {
        while (fabs(fval)>PREC && fabs(fval/g)>PREC) {
            /*std::cout<<"s1 = "<<s1<<" s2 = "<<s2<<std::endl;
            std::cout<<"a = "<<a<<" b = "<<b<<" c = "<<c<<" d = "<<d<<" e = "<<e<<std::endl;
            std::cout<<"fval5 = "<<fval<<"  "<<g<<std::endl;
            std::cout<<"x = "<<x<<std::endl;*/
            x -= fval/g;
            fval = df(x);
            g = ddf(x);
        }
        has_solution[0] = true;
        solution[0] = x;
        ++solution_count;
    }


    /* Solve to the right of s2*/
    x = s2+EPS;
    fval = df(x);
    g = ddf(x);

    if ( (fval>0 && g <0) || (fval<0 && g > 0) ) {
        while (fabs(fval)>PREC && fabs(fval/g)>PREC) {
     //       std::cout<<"fval6 = "<<fval<<"  "<<g<<std::endl;
            x -= fval/g;
            fval = df(x);
            g = ddf(x);
        }
        has_solution[1] = true;
        solution[1] = x;
        ++solution_count;
    }

    /* Solve between s1 and s2*/

    if (s1 != s2) {
        double f1 = df(s1);
        double f2 = df(s2);

        if (f1*f2<0) { //They are on opposite sides of 0
            x = (s2+s1)/2;
            fval = df(x);
            while (fabs(fval)>PREC && fabs(s2-s1)>PREC) {
       //         std::cout<<"fval7 = "<<fval<<"  "<<s2-s1<<std::endl;
                if (f1*fval<0) {
                    s2 = x;
                    f2 = fval;
                } else {
                    s1 = x;
                    f1 = fval;
                }
                x = (s2+s1)/2;
                fval = df(x);
            }
            has_solution[2] = true;
            solution[2] = x;
            ++solution_count;
        }
    } else {
        if (fabs(df(s1)) < PREC) {
            has_solution[2] = true;
            solution[2] = s1;
            ++solution_count;
        }
    }



    std::vector<double> result;
    for (int i = 0; i<3; i++)
        if (has_solution[i])
            result.push_back(solution[i]);

    return result;

}

std::vector<double> PieperSolver::ddfSolve() const {
    double ac = 12*a;
    double bc = 6*b;
    double cc = 2*c;

    double dc = bc*bc-4*ac*cc;
    if (dc < 0) {
        return std::vector<double>();
    } else {
        std::vector<double> result;
        double res = (-bc + sqrt(dc))/(2*ac);
        result.push_back(res);
        res = (-bc - sqrt(dc))/(2*ac);
        result.push_back(res);
        return result;
    }
}



void PieperSolver::setupCoefficients(double r, double z) const {

    a = (Power(a1,2) + Power(Power(a2 - a3,2) + Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 +
                             Power(d4,2) - r + 2*d2*(calpha2*(d3 + calpha3*d4) + d4*salpha2*salpha3),2)/Power(a1,2) +
         (2*(2*Power(calpha1,2)*Power(d2 + calpha2*(d3 + calpha3*d4) + d4*salpha2*salpha3,2) +
             Power(salpha1,2)*(-Power(a2 - a3,2) + Power(d2,2) - r +
                               Power(d3,2)*(1 - 2*Power(salpha2,2)) +
                               2*d3*d4*(calpha3 - 2*calpha3*Power(salpha2,2) + 2*calpha2*salpha2*salpha3) +
                               2*d2*(calpha2*(d3 + calpha3*d4) + d4*salpha2*salpha3) +
                               Power(d4,2)*(1 - 2*Power(calpha3*salpha2 - calpha2*salpha3,2))) -
             4*calpha1*(d2 + calpha2*(d3 + calpha3*d4) + d4*salpha2*salpha3)*z + 2*Power(z,2)))/
         Power(salpha1,2))/4.;

    b = (2*(Power(salpha1,2)*(a3*d2*salpha2 + a2*d4*salpha3)*
            (Power(a2 - a3,2) + Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) - r +
             2*d2*(calpha2*(d3 + calpha3*d4) + d4*salpha2*salpha3)) +
            Power(a1,2)*(-(a2*d4*Power(salpha1,2)*salpha3) +
                         a3*(Power(salpha1,2)*((d2 + 2*calpha2*(d3 + calpha3*d4))*salpha2 -
                                               2*(-1 + Power(calpha2,2))*d4*salpha3) +
                             2*Power(calpha1,2)*salpha2*
                             (d2 + calpha2*d3 + calpha2*calpha3*d4 + d4*salpha2*salpha3) - 2*calpha1*salpha2*z))))
        /(Power(a1,2)*Power(salpha1,2));

    c = Power(a1,2)/2. - Power(a2,2) + Power(a3,2)*(3 - 4*Power(calpha2,2)) + Power(d2,2) + Power(d3,2) +
        2*calpha3*d3*d4 + Power(d4,2) + 2*calpha2*d2*(d3 + calpha3*d4) - r -
        2*Power(d3,2)*Power(salpha2,2) - 4*calpha3*d3*d4*Power(salpha2,2) -
        2*Power(calpha3,2)*Power(d4,2)*Power(salpha2,2) +
        2*(-2 + Power(calpha2,2))*Power(d4,2)*Power(salpha3,2) +
        (2*Power(calpha1,2)*(Power(d2 + calpha2*(d3 + calpha3*d4),2) + 2*Power(a3,2)*Power(salpha2,2) -
                             Power(d4,2)*Power(salpha2,2)*Power(salpha3,2)))/Power(salpha1,2) +
        (Power(a2,4) + Power(a3,4) + 2*Power(a3,2)*
         (Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) + 2*calpha2*d2*(d3 + calpha3*d4) - r +
          Power(d2,2)*(1 + 4*Power(salpha2,2))) + 24*a2*a3*d2*d4*salpha2*salpha3 +
         2*Power(a2,2)*(-Power(a3,2) + Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) +
                        2*calpha2*d2*(d3 + calpha3*d4) - r + 4*Power(d4,2)*Power(salpha3,2)) +
         (Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) - r +
          2*d2*(calpha2*d3 + calpha2*calpha3*d4 - d4*salpha2*salpha3))*
         (Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) - r +
          2*d2*(calpha2*d3 + calpha2*calpha3*d4 + d4*salpha2*salpha3)))/(2.*Power(a1,2)) -
        (4*calpha1*(d2 + calpha2*(d3 + calpha3*d4))*z)/Power(salpha1,2) +
        (2*Power(z,2))/Power(salpha1,2);

    d = (2*(Power(salpha1,2)*(a3*d2*salpha2 + a2*d4*salpha3)*
            (Power(a2 + a3,2) + Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 + Power(d4,2) - r +
             2*d2*(calpha2*(d3 + calpha3*d4) - d4*salpha2*salpha3)) +
            Power(a1,2)*(-(a2*d4*Power(salpha1,2)*salpha3) +
                         a3*(Power(salpha1,2)*((d2 + 2*calpha2*(d3 + calpha3*d4))*salpha2 +
                                               2*(-1 + Power(calpha2,2))*d4*salpha3) +
                             2*Power(calpha1,2)*salpha2*
                             (d2 + calpha2*d3 + calpha2*calpha3*d4 - d4*salpha2*salpha3) - 2*calpha1*salpha2*z))))
        /(Power(a1,2)*Power(salpha1,2));

    e = (Power(a1,2) + Power(Power(a2 + a3,2) + Power(d2,2) + Power(d3,2) + 2*calpha3*d3*d4 +
                             Power(d4,2) - r + 2*d2*(calpha2*(d3 + calpha3*d4) - d4*salpha2*salpha3),2)/Power(a1,2) +
         (2*(2*Power(calpha1,2)*Power(d2 + calpha2*(d3 + calpha3*d4) - d4*salpha2*salpha3,2) -
             Power(salpha1,2)*(Power(a2 + a3,2) - Power(d2,2) - 2*calpha2*d2*(d3 + calpha3*d4) + r +
                               Power(d3,2)*(-1 + 2*Power(salpha2,2)) + 2*d2*d4*salpha2*salpha3 +
                               2*d3*d4*(calpha3*(-1 + 2*Power(salpha2,2)) + 2*calpha2*salpha2*salpha3) +
                               Power(d4,2)*(-1 + 2*Power(calpha3*salpha2 + calpha2*salpha3,2))) -
             4*calpha1*(d2 + calpha2*(d3 + calpha3*d4) - d4*salpha2*salpha3)*z + 2*Power(z,2)))/
         Power(salpha1,2))/4.;
}

PieperSolver::~PieperSolver() {
	// TODO Auto-generated destructor stub
}

} /* namespace ik */
} /* namespace robot */
