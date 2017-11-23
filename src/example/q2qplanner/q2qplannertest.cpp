/*
 * q2qplannertest.cpp
 *
 *  Created on: Nov 21, 2017
 *      Author: a1994846931931
 */

# include "q2qplannertest.h"
# include "../../pathplanner/QtoQPlanner.h"
# include "../../math/Q.h"
# include <time.h>
# include "../../common/printAdvance.h"
# include <fstream>
# include "../../simulation/MotionStack.h"
# include "../../trajectory/Sampler.h"
# include "../../common/fileAdvance.h"

using namespace robot::common;
using namespace robot::pathplanner;
using namespace robot::trajectory;
using namespace robot::simulation;

void q2qplannertest()
{
	// Point to point planner
	Q h = Q(100, 100, 100, 100, 100, 100);
	Q aMax = Q(30, 30, 30, 30, 50, 50);
	Q vMax = Q(1.5, 1.5, 1.5, 1.5, 1, 1);
	Q start = Q(0, 0, 0, 0, 0, 0);
	Q end = Q(2, 0.5, 0.5, 0, -1.2, 2);
	QtoQPlanner planner = QtoQPlanner(vMax, aMax, start, end);
	clock_t clockStart = clock();
	Interpolator<Q>::ptr qIpr = planner.query();
	clock_t clockEnd = clock();
	cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
	double T = qIpr->duration();
	cout << "总时长: " << T << "s" << endl;
	std::vector<Q> x;
	std::vector<Q> dx;
	std::vector<Q> ddx;
	const int count = 200;
	clockStart = clock();
	x = Sampler<Q>::sample(qIpr, count, "x");
	dx = Sampler<Q>::sample(qIpr, count, "dx");
	ddx = Sampler<Q>::sample(qIpr, count, "ddx");
	clockEnd = clock();
	cout << "每次插补用时: " << (clockEnd - clockStart)/(double)count << "us" << endl;

	/**> 保存文件 */
	vector<double> vt = Sampler<double>::linspace(0, T, count);
	saveQPath("src/example/q2qplanner/tempx.txt", x, vt);
}
