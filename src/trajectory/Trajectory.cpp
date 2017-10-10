/*
 * Trajectory.cpp
 *
 *  Created on: Sep 29, 2017
 *      Author: a1994846931931
 */

#include "Trajectory.h"

namespace robot {
namespace trajectory {

Trajectory::Trajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
		std::shared_ptr<robot::ik::IKSolver> iksolver,
		robot::model::Config config): ikInterpolator(origin, iksolver, config)
{

}


Trajectory::qVelAcc Trajectory::sampleVelAcc(const int count, double precision)
{
	vector<Q> dq;
	vector<Q> ddq;
	double L = this->duration();
	double dl = L/(double)(count - 1);
	Q tempQ1 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(0), _rotInterpolator->x(0)), _config)[0];
	Q tempQ2 = Q::zero(tempQ1.size());
	Q tempQ3 = Q::zero(tempQ1.size());
	for (double l=0; l<=L; l+=dl)
	{
		Q tempQ1 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(l), _rotInterpolator->x(l)), _config)[0];
		Q tempQ2 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(l + precision), _rotInterpolator->x(l + precision)), _config)[0];
		Q tempQ3 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(l + precision*2), _rotInterpolator->x(l + precision*2)), _config)[0];
		dq.push_back((tempQ2 - tempQ1)/precision);
		ddq.push_back((tempQ3 - tempQ2*2 + tempQ1)/(precision*precision));
	}
	qVelAcc result;
	result.dq = dq;
	result.ddq = ddq;
	return result;
}

vector<Q> Trajectory::sampleVel(const int count, double precision)
{
	vector<Q> dq;
	double L = this->duration();
	double dl = L/(double)(count - 1);
	Q tempQ1 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(0), _rotInterpolator->x(0)), _config)[0];
	Q tempQ2 = Q::zero(tempQ1.size());
	for (double l=0; l<=L; l+=dl)
	{
		Q tempQ1 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(l), _rotInterpolator->x(l)), _config)[0];
		Q tempQ2 = _ikSolver->solve(HTransform3D<double>(_posInterpolator->x(l + precision), _rotInterpolator->x(l + precision)), _config)[0];
		dq.push_back((tempQ2 - tempQ1)/precision);
	}
	return dq;
}

std::pair<std::pair<Q, Q>, std::pair<Q, Q> > Trajectory::getMinMax(const int count, double precision)
{
	qVelAcc result = this->sampleVelAcc(count, precision);
	vector<Q> dq = result.dq;
	vector<Q> ddq = result.ddq;
	Q dqMin = dq[0];
	Q dqMax = dqMin;
	Q ddqMin = ddq[0];
	Q ddqMax = ddqMin;
	for_each(dq.begin(), dq.end(), [&](Q& q){q.doMinmax(dqMin, dqMax);});
	for_each(ddq.begin(), ddq.end(), [&](Q& q){q.doMinmax(ddqMin, ddqMax);});
	return std::make_pair(std::make_pair(dqMin, dqMax), std::make_pair(ddqMin, ddqMax));
}

std::pair<Q, Q> Trajectory::getMaxAbs(const int count, double precision)
{
	qVelAcc result = this->sampleVelAcc(count, precision);
	vector<Q> dq = result.dq;
	vector<Q> ddq = result.ddq;
	Q dqMax = dq[0];
	Q ddqMax = ddq[0];
	for_each(dq.begin(), dq.end(), [](Q& q){q.abs();});
	for_each(ddq.begin(), ddq.end(), [](Q& q){q.abs();});
	for_each(dq.begin(), dq.end(), [&](Q& q){q.doMax(dqMax);});
	for_each(ddq.begin(), ddq.end(), [&](Q& q){q.doMax(ddqMax);});
	return std::make_pair(dqMax, ddqMax);
}

vector<double> Trajectory::sampleMaxSpeed(const int count, Q dqMax, double precision)
{
	vector<Q> dq = sampleVel(count, precision);
	for_each(dq.begin(), dq.end(), [](Q& q){q.abs();});
	vector<double> maxSpeed;
	for (int i=0; i<(int)dq.size(); i++)
	{
		maxSpeed.push_back((dqMax/dq[i]).getMin());
	}
	return maxSpeed;
}

double Trajectory::getMaxSpeed(const int count, Q dqMax, double v, double precision)
{
	vector<Q> dq = sampleVel(count, precision);
	for_each(dq.begin(), dq.end(), [](Q& q){q.abs();});
	double maxSpeed = v;
	double tempSpeed = 0;
	for (int i=0; i<(int)dq.size(); i++)
	{
		tempSpeed = (dqMax/dq[i]).getMin();
		maxSpeed = tempSpeed < maxSpeed ? tempSpeed:maxSpeed;
	}
	return maxSpeed;
}

double Trajectory::getMaxSpeed(const int count, Q dqMax, Q ddqMax, double v, double precision)
{
	Trajectory::qVelAcc velAcc = sampleVelAcc(count, precision);
	vector<Q> dq = velAcc.dq;
	vector<Q> ddq = velAcc.ddq;
	for_each(dq.begin(), dq.end(), [](Q& q){q.abs();});
	for_each(ddq.begin(), ddq.end(), [](Q& q){q.abs();});
	double maxSpeed = v;
	double tempSpeed = 0;
	for (int i=0; i<(int)dq.size(); i++)
	{
		tempSpeed = (dqMax/dq[i]).getMin();
		maxSpeed = tempSpeed < maxSpeed ? tempSpeed:maxSpeed;
		tempSpeed = sqrt((ddqMax/ddq[i]).getMin());
		maxSpeed = tempSpeed < maxSpeed ? tempSpeed:maxSpeed;
	}
	return maxSpeed;
}

} /* namespace trajectory */
} /* namespace robot */
