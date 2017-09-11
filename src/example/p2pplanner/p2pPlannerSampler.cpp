/*
 * p2pPlannerSampler.cpp
 *
 *  Created on: Sep 11, 2017
 *      Author: a1994846931931
 */

# include "p2pPlannerSampler.h"
# include "../../pathplanner/PointToPointPlanner.h"
# include "../../math/Q.h"
# include <time.h>
# include "../../common/printAdvance.h"
# include <fstream>

using namespace robot::common;
using namespace robot::pathplanner;
using namespace robot::trajectory;

void p2pPlannerSampler()
{
	// Point to point planner
	Q h = Q(100, 100, 100, 100, 100, 100);
	Q aMax = Q(30, 30, 30, 30, 50, 50);
	Q vMax = Q(1.5, 1.5, 1.5, 1.5, 1, 1);
	Q start = Q(0, 0, 0, 0, 0, 0);
	Q end = Q(2, 0.5, 0.5, 0, -1.2, 2);
	PointToPointPlanner planner = PointToPointPlanner(h, aMax, vMax);
	clock_t clockStart = clock();
	Interpolator<Q>* p2pPlanner = planner.query(start, end);
	clock_t clockEnd = clock();
	cout << "插补器构造用时: " << clockEnd - clockStart << "us" << endl;
	int step = 100;
	double T = p2pPlanner->duration();
	double dt = T/(step - 1);
	cout << "总时长: " << T << "s" << endl;
	std::vector<Q> x;
	std::vector<Q> dx;
	std::vector<Q> ddx;
	clockStart = clock();
	for (double t=0; t<T; t+=dt)
	{
		x.push_back(p2pPlanner->x(t));
		dx.push_back(p2pPlanner->dx(t));
		ddx.push_back(p2pPlanner->ddx(t));
	}
	clockEnd = clock();
	cout << "插补用时: " << clockEnd - clockStart << "us" << endl;

	/**> 保存文件 */
	const char* filename1 = "src/example/p2pplanner/tempx.txt";
	std::ofstream out1(filename1);
	/**> Data for Matlab */
	out1 << "[" << endl;
	for (int i=0; i<(int)x.size(); i++)
	{
		out1 << x[i][0] << ", " << x[i][1] << ", " << x[i][2] << ", " << x[i][3] << ", " << x[i][4] << ", " << x[i][5] << ";" << endl;
	}
	out1 << "]" << endl;
	out1.close();
}

