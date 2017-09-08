/*
 * PointToPointPlanner.cpp
 *
 *  Created on: Sep 5, 2017
 *      Author: a1994846931931
 */

# include "PointToPointPlanner.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../trajectory/ConvertedInterpolator.h"

namespace robot {
namespace pathplanner {

PointToPointPlanner::PointToPointPlanner(Q h, Q aMax, Q vMax)
{
	_h = h;
	_aMax = aMax;
	_vMax = vMax;
	_size = _h.size();
	if (_aMax.size() != _size || _vMax.size() != _size)
		throw ("错误: 点对点规划器必须由同样大小的Q进行构造!");
}

Interpolator<Q>* PointToPointPlanner::querry(Q qStart, Q qEnd)
{
	Q distance = qStart - qEnd;
	vector<Interpolator<double>* > seprateInterpolator;
	vector<Interpolator<double>* > qInterpolators;
	double tMax = 0;
	for (int i=0; i<_size; i++)
	{
		seprateInterpolator.push_back(_smPlanner.query(distance[i], _h[i], _aMax[i], _vMax[i], qStart[i]));
		double duration = (*(seprateInterpolator.end() - 1))->duration();
		tMax = (tMax > duration) ? tMax:duration;
	}
	for (int i=0; i<_size; i++)
	{
		LinearCompositeInterpolator<double>* lcI =
				new LinearCompositeInterpolator<double>(seprateInterpolator[i], seprateInterpolator[i]->duration()/tMax);
		qInterpolators.push_back(lcI);
		_interpolatorList.push_back(lcI);
	}
	ConvertedInterpolator<std::vector<Interpolator<double>* > , robot::math::Q>* qInterpolator =
			new ConvertedInterpolator<std::vector<Interpolator<double>* > , robot::math::Q>(qInterpolators);
	_qInterpolatorList.push_back(qInterpolator);
	return qInterpolator;
}

PointToPointPlanner::~PointToPointPlanner()
{
	for (int i=0; i<(int)_interpolatorList.size(); i++)
		delete _interpolatorList[i];
	for (int i=0; i<(int)_qInterpolatorList.size(); i++)
		delete _qInterpolatorList[i];
}

} /* namespace pathplanner */
} /* namespace robot */
