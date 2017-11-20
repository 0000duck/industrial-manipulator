/*
 * QtoQPlanner.cpp
 *
 *  Created on: Oct 27, 2017
 *      Author: a1994846931931
 */

#include "QtoQPlanner.h"
# include "../common/common.h"
# include "SmoothMotionPlanner.h"
# include "SMPlannerEx.h"

using namespace robot::common;

namespace robot {
namespace pathplanner {

QtoQPlanner::QtoQPlanner(Q dqLim, Q ddqLim,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		Q start, Q qEnd)
: _dqLim(dqLim), _ddqLim(ddqLim),
  _ikSolver(ikSolver), _serialLink(ikSolver->getRobot()),
  _qStop(start), _qEnd(qEnd), _size(start.size())
{
	_v = 10;
	_a = 30;
	_h = 100;
	if (_qStop == _qEnd)
		throw("错误<QtoQPlanner>: 始末的关节数值不能相同!");
	_k = vector<double>(_size, 0);
}

void QtoQPlanner::query()
{
	Q distances = _qEnd - _qStop;
	double L = (fabs(distances.getMin()) > fabs(distances.getMax())) ? fabs(distances.getMin()):fabs(distances.getMax());
	for (int i=0; i<_size; i++)
	{
		_k[i] = distances[i]/L;
		if (fabs(_k[i]) < 1e-12)
			continue;
		_v = common::min_d(_v, _dqLim[i]/_k[i]);
		_a = common::min_d(_a, _ddqLim[i]/_k[i]);
		// h??
	}

	SmoothMotionPlanner planner;

	_lt = planner.query(L, _h, _a, _v, 0);

	_ltoqIpr.clear();
	for (int i=0; i<_size; i++)
	{
		if (fabs(_k[i]) > 1e-12)
		{
			_ltoqIpr.push_back(std::make_shared<ConvertedInterpolator<double, double> >(
					_lt,
					[=](double t){return _qStop[i] + _k[i]*(_lt->x(t));},
					[=](double t){return _k[i]*(_lt->dx(t));},
					[=](double t){return _k[i]*(_lt->ddx(t));}));
		}
		else
		{
			_ltoqIpr.push_back(std::make_shared<ConvertedInterpolator<double, double> >(
					_lt,
					[=](double t){return _qStop[i];},
					[=](double t){return 0;},
					[=](double t){return 0;}));
		}
	}
	_qIpr = std::make_shared<ConvertedInterpolator<std::vector<Interpolator<double>::ptr > , robot::math::Q> >(_ltoqIpr);
}

void QtoQPlanner::doQuery()
{
	query();
}

bool QtoQPlanner::stop(double t, Interpolator<Q>::ptr& stopIpr)
{
	if (_qIpr.get() == NULL)
		throw("错误<QtoQPlanner>: 尚未进行规划!\n");
	double x0 = _lt->x(t);
	double v0 = _lt->dx(t);
	double a0 = _lt->ddx(t);
	double remain = _lt->end() - x0;
	SMPlannerEx planner;
	Interpolator<double>::ptr stopLt = planner.query_stop(x0, v0, a0, _h, _a);
	/**> 判断剩余距离是否足够停止 */
	if ((stopLt->end() - x0) >= remain)
	{
		cout << "错误<LinePlanner>: 距离不够, 无法停止!\n";
		return false;
	}
	std::vector<Interpolator<double>::ptr> ltoqIpr;
	for (int i=0; i<_size; i++)
	{
		if (fabs(_k[i]) > 1e-12)
		{
			ltoqIpr.push_back(std::make_shared<ConvertedInterpolator<double, double> >(
					_lt,
					[=](double t){return _qStop[i] + _k[i]*(stopLt->x(t));},
					[=](double t){return _k[i]*(stopLt->dx(t));},
					[=](double t){return _k[i]*(stopLt->ddx(t));}));
		}
		else
		{
			ltoqIpr.push_back(std::make_shared<ConvertedInterpolator<double, double> >(
					_lt,
					[=](double t){return _qStop[i];},
					[=](double t){return 0;},
					[=](double t){return 0;}));
		}
	}
	stopIpr = std::make_shared<ConvertedInterpolator<std::vector<Interpolator<double>::ptr > , robot::math::Q> >(ltoqIpr);
	_qStop = stopIpr->end();
	return true;
}

void QtoQPlanner::resume()
{
	query();
}

bool QtoQPlanner::isTrajectoryExist() const
{
	if (_qIpr.get() == NULL)
		return false;
	return true;
}

Interpolator<Q>::ptr QtoQPlanner::getQTrajectory() const
{
	return _qIpr;
}

} /* namespace pathplanner */
} /* namespace robot */
