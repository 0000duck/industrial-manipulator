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
	virtual ~PieperSolver();
private:
	robot::model::DHTable _dHTable;


};

} /* namespace ik */
} /* namespace robot */

#endif /* PIEPERSOLVER_H_ */
