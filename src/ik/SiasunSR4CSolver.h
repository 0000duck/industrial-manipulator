/*
 * SiasunSR4CSolver.h
 *
 *  Created on: Aug 28, 2017
 *      Author: a1994846931931
 */

#ifndef SIASUNSR4CSOLVER_H_
#define SIASUNSR4CSOLVER_H_

# include <vector>
# include "../math/Q.h"
# include "../model/DHTable.h"
# include "../model/SerialLink.h"
# include "../model/Config.h"

namespace robot {
namespace ik {

class SiasunSR4CSolver {
public:
	SiasunSR4CSolver(robot::model::SerialLink& serialRobot);
    std::vector<Q> solve(const HTransform3D<>& baseTend, const model::Config&) const;
    void solveTheta456(double theta1,
                       double theta2,
                       double theta3,
                       robot::math::HTransform3D<>& T06,
                       std::vector<robot::math::Q>& result,
                       const model::Config&) const;
    bool isValid() const; // TODO

	bool isShoulderValid(const robot::math::Q&, const model::Config&) const;
	bool isShoulderValid(const double r, const model::Config&) const;
	bool isElbowValid(const robot::math::Q&, const model::Config&) const;
	bool isElbowValid(const double j3, const model::Config&) const;
	bool isWristValid(const robot::math::Q&, const model::Config&) const;
	bool isWristValid(const double j5, const model::Config&) const;
	virtual ~SiasunSR4CSolver();
private:
    double _alpha1, _a1, _calpha1, _salpha1, _d1;
    double _alpha2, _a2, _calpha2, _salpha2, _d2;
    double _alpha3, _a3, _calpha3, _salpha3, _d3;
    double _alpha4, _a4, _calpha4, _salpha4, _d4;
    double _alpha5, _a5, _calpha5, _salpha5, _d5;
    double _alpha6, _a6, _calpha6, _salpha6, _d6;
private:
	robot::model::DHTable _dHTable;
    robot::math::HTransform3D<> _0Tbase;
    robot::math::HTransform3D<> _endTjoint6;
    robot::model::SerialLink* _serialLink;
};

} /* namespace ik */
} /* namespace robot */

#endif /* SIASUNSR4CSOLVER_H_ */
