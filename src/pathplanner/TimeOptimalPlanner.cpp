/*
 * TimeOptimalPlanner.cpp
 *
 *  Created on: Nov 7, 2017
 *      Author: a1994846931931
 */

#include "TimeOptimalPlanner.h"
# include "SMPlannerEx.h"
# include <stack>

namespace robot {
namespace pathplanner {

TimeOptimalPlanner::TimeOptimalPlanner() {
	// TODO Auto-generated constructor stub

}

TimeOptimalPlanner::~TimeOptimalPlanner() {
	// TODO Auto-generated destructor stub
}

void TimeOptimalPlanner::optimizeVelocityRestriction(vector<double>& maxSpeed, double a, double h, double ds)
{
	std::stack<int> checkIndex;
	SMPlannerEx planner;
	maxSpeed[maxSpeed.size() - 1] = 0;
	double trueV;
	for (int i=0; i<maxSpeed.size(); i++)
		checkIndex.push(i);
	while(!checkIndex.empty())
	{
		int index = checkIndex.top();
		checkIndex.pop();
		if (index > 0 && maxSpeed[index] < maxSpeed[index - 1])
		{
			if (! planner.checkDitance(ds, h, a, maxSpeed[index], maxSpeed[index - 1], trueV)) //index - 1处速度无法达到
			{
				cout << ds << " " << h << " " << a << " " << maxSpeed[index] << " " << maxSpeed[index - 1] << " " << trueV << endl;
				maxSpeed[index - 1] = trueV;
				checkIndex.push(index - 1);
				cout << trueV << endl;
			}
		}
		if (index < maxSpeed.size() - 1 && maxSpeed[index] < maxSpeed[index + 1])
		{
			if (! planner.checkDitance(ds, h, a, maxSpeed[index], maxSpeed[index + 1], trueV)) //index - 1处速度无法达到
			{
				maxSpeed[index + 1] = trueV;
				checkIndex.push(index + 1);
			}
		}
	}

}

SequenceInterpolator<double>::ptr TimeOptimalPlanner::getOptimalLt(std::function<double(double)>& Vm, double s, double ve, double a, double h, double ds)
{
	SMPlannerEx planner;
	auto seqIpr = std::make_shared<SequenceInterpolator<double> >();
	double dt = ds/ve; //理论上可以防止跳过某一低速区
	double vs = 0; //初始速度
	double l0 = 0; //初始移动距离
	SequenceInterpolator<double>::ptr curIpr = planner.query_flexible(l0, s - l0, h, a, vs, ve, ve, true); //查询初始规划, v若无法达到,改为实际的值
	double tCheck = dt;
	bool checkFinished = false;
	while(!checkFinished)
	{
		while(tCheck < curIpr->duration())
		{
			double vMax = Vm(curIpr->x(tCheck));
			if (vMax <= 0)
				throw("错误<TimeOptimalPlanner>: 出现限速极小情况!");
			if (curIpr->dx(tCheck) <= vMax) //速度合适, 执行下一个检查点
			{
				tCheck += dt;
			}
			else //速度不合适
			{
				double l1 = 0;
				if (tCheck > dt)
				{
					double tempv;
					l1 = curIpr->x(tCheck - dt);
					seqIpr->addInterpolator(
							planner.query_flexible(l0, l1 - l0, h, a, vs, ve, vMax, tempv, vs)); //三速度柔性规划
					l0 = l1;
				}
				l1 = curIpr->x(tCheck);
				seqIpr->addInterpolator(
						planner.query_flexible(l0, l1 - l0, h, a, vs, vMax, vs)); //二速度柔性规划, 若tCheck > dt, 应为匀速规划
				l0 = l1;
				curIpr = planner.query_flexible(l0, s - l0, h, a, vs, ve, ve, true);
				tCheck = dt;
			}
		}
		seqIpr->addInterpolator(curIpr);
		checkFinished = true;
	}
	return seqIpr;
}

SequenceInterpolator<double>::ptr TimeOptimalPlanner::getOptimalLt(vector<double> optimizedMaxSpeed, double s, double ve, double a, double h, double ds)
{
	SMPlannerEx planner;
	auto seqIpr = std::make_shared<SequenceInterpolator<double> >();
	double vs = 0; //初始速度
	double l0 = 0; //初始移动距离
	for (int i=1; i<(int)optimizedMaxSpeed.size(); i++)
	{
		if (i == (int)optimizedMaxSpeed.size() - 1)
			seqIpr->addInterpolator(
					planner.query_flexible(l0, ds, h, a, vs, optimizedMaxSpeed[i], vs, true));
		else
			seqIpr->addInterpolator(
					planner.query_flexible(l0, ds, h, a, vs, optimizedMaxSpeed[i], vs));
		l0 += ds;
	}
	return seqIpr;
}

} /* namespace pathplanner */
} /* namespace robot */
