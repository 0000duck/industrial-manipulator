/*
 * PieperSolver.h
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#ifndef PIEPERSOLVER_H_
#define PIEPERSOLVER_H_

# include <vector>
# include "../math/Q.h"
# include "../model/DHTable.h"
# include "../model/SerialLink.h"

namespace robot {
namespace ik {

class PieperSolver {
public:
	PieperSolver(robot::model::SerialLink& serialRobot);
	std::vector<robot::math::Q> solve(const robot::math::HTransform3D<>& T06);
	virtual ~PieperSolver();
private:
	robot::model::DHTable _dHTable;
	void init();
	void solveTheta456(double theta1,
					   double theta2,
					   double theta3,
					   const robot::math::HTransform3D<>& T06,
					   std::vector<robot::math::Q>& result) const;
	/**
	 * @brief Solves the case with a1 = 0 (Case 1 in Craig)
	 */
	std::vector<double> solveTheta3Case1(double z) const;

	/**
	 * @brief Solves the case with a1 != 0 and sin(alpha1) = 0 (Case 2 in Craig)
	 */
	std::vector<double> solveTheta3Case2(double r) const;

	/**
	 * @brief Solves the general case with a1 != 0 and sin(alpha1) != 0 (Case 3 in Craig)
	 */
	std::vector<double> solveTheta3Case3(double r, double z) const;


	double solveTheta2(double r, double z, double theta3) const;

	std::vector<double> solveTheta2Case1(double z, double theta3) const;
	std::vector<double> solveTheta2Case2(double r, double theta3) const;

	double solveTheta1(double x, double y, double theta2, double theta3) const;

	double f(double x) const;
	double df(double x) const;
	double ddf(double x) const;

	std::vector<double> fSolve() const;
	std::vector<double> fSolve(double s1, double s2, double s3) const;
	std::vector<double> dfSolve(double s1, double s2) const;
	std::vector<double> ddfSolve() const;

	void setupCoefficients(double r, double z) const;


	/*
	 * Variables used for calculating
	 * Defined here to avoid allocating memory for them all the time
	 */
	mutable double a,b,c,d,e;

	mutable double alpha0, a0, calpha0, salpha0, d1;
	mutable double alpha1, a1, calpha1, salpha1, d2;
	mutable double alpha2, a2, calpha2, salpha2, d3;
	mutable double alpha3, a3, calpha3, salpha3, d4;
	mutable double alpha4, a4, calpha4, salpha4, d5;
	mutable double alpha5, a5, calpha5, salpha5, d6;

};

} /* namespace ik */
} /* namespace robot */

#endif /* PIEPERSOLVER_H_ */
