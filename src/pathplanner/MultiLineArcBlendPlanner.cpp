/*
 * MultiLineArcBlendPlanner.cpp
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#include "MultiLineArcBlendPlanner.h"
# include "../math/Quaternion.h"
# include "../model/Config.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/LinearInterpolator.h"
# include "../trajectory/CompositeInterpolator.h"
# include "../common/printAdvance.h"
# include "../pathplanner/SMPlannerEx.h"
# include <algorithm>

using robot::math::Quaternion;
using robot::model::Config;
using namespace robot::common;

namespace robot {
namespace pathplanner {

MultiLineArcBlendPlanner::MultiLineArcBlendPlanner(Q dqLim, Q ddqLim,
		std::shared_ptr<robot::ik::IKSolver> ikSolver,
		const vector<Q>& Qpath, const vector<double>& arcRatio, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk) :
	_ikSolver(ikSolver), _serialLink(ikSolver->getRobot()), _qMin(_serialLink->getJointMin()), _qMax(_serialLink->getJointMax()), _dqLim(dqLim), _ddqLim(ddqLim)
{
	_startFromLine = true;
	_size = _serialLink->getDOF();
	if (_qMin.size() != _size || _qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<多线段圆弧混合规划器>:　构造参数中数组的长度不一致！");

	int num = (int)Qpath.size() - 1; //线段的数量
	if (num != (int)arcRatio.size() + 1 || num != (int)velocity.size() || num != (int)acceleration.size() ||num != (int)jerk.size())
		throw ("错误<多线段圆弧混合规划器>:　构造参数的长度不一致！");

	_qStop = Qpath[0];
	_velocity.push_back(velocity[0]);
	_acceleration.push_back(acceleration[0]);
	_jerk.push_back(jerk[0]);
	for (int i=1; i<num; i++)
	{
		_velocity.push_back(velocity[i - 1] > velocity[i] ? velocity[i - 1] : velocity[i]);
		_velocity.push_back(velocity[i]);
		_acceleration.push_back(acceleration[i - 1] > acceleration[i] ? acceleration[i - 1] : acceleration[i]);
		_acceleration.push_back(acceleration[i]);
		_jerk.push_back(jerk[i - 1] > jerk[i] ? jerk[i - 1] : jerk[i]);
		_jerk.push_back(jerk[i]);
	}

	vector<std::pair<Rotation3D<double>, Rotation3D<double> > > lineRotation; /** 直线段上的关键姿态 (n-1)*2 */
	vector<std::pair<Vector3D<double>, Vector3D<double> > > linePosition; /** 直线段上的关键点 (n-1)*2 */
	vector<Vector3D<double> > arcMidPosition; /** 圆弧中间点 n-2 */
	vector<Rotation3D<double> > arcMidRotation; /** 圆弧中间点 n-2 */

	/** n */
	int pathSize = (int)Qpath.size();
	/** @todo 共线检查 */
	/** n-1 */
	int lineSize = pathSize - 1;
	if (lineSize < 2)
		throw("错误<MultiLineArcBlendPlanner>: 路径点数量不足, 至少要三个点!");
	/** n-2 */
	int arcSize = lineSize - 1;
	if ((int)arcRatio.size() < arcSize)
		throw("错误<MultiLineArcBlendPlanner>: 圆弧比例参数数量不足!");

	/**> 检查config参数 */
	_config = _ikSolver->getConfig(Qpath[0]);
	for (int i=1; i<pathSize; i++)
		if (_config != _ikSolver->getConfig(Qpath[i]))
		{
			cout << "初始Config: \n" ;
			_config.print();
			cout << "该关节Config: \n" ;
			(_ikSolver->getConfig(Qpath[i])).print();
			throw (std::string("错误<MultiLineArcBlendPlanner>: 第 ") + to_string(i + 1) + std::string(" 个关节的的Config和初始点不同!"));
		}

	/**> 保存路径点 */
	vector<HTransform3D<double> > path; /** 路径点 n */
	for (int i=0; i<pathSize; i++)
		path.push_back(_serialLink->getEndTransform(Qpath[i]));

	/**> 记录线段长度 */
	vector<double> lineLength;
	for (int i=0; i<lineSize; i++)
		lineLength.push_back((path[i].getPosition() - path[i + 1].getPosition()).getLength());

	/**> 记录修正比例因子 */
	vector<std::pair<double, double> > modifiedRatio; /** 修正插补比例因子 (n-2)*2 */
	for (int i=0; i<arcSize; i++)
	{
		double l1 = lineLength[i];
		double l2 = lineLength[i+1];
//		cout << "l1 = " << l1 << " l2 = " << l2 << endl;
		double ratiol = arcRatio[i];
		double ratior = ratiol;
		if (l1 < l2)
			ratior = ratiol*l1/l2;
		else if(l1 > l2)
			ratiol = ratior*l2/l1;
		modifiedRatio.push_back(std::pair<double, double>(1.0 - ratiol, ratior));
	}

	/**> 记录线段的始末位姿 */
	lineRotation.push_back(std::pair<Rotation3D<double>, Rotation3D<double> >(
			path[0].getRotation(),
			Quaternion::interpolate(path[0].getRotation(), path[1].getRotation(), modifiedRatio[0].first)));
	double k1 = 0;
	double k2 = modifiedRatio[0].first;
//	cout << "k2 = " << k2 << endl;
	linePosition.push_back(std::pair<Vector3D<double>, Vector3D<double> >(
			path[0].getPosition(),
			path[0].getPosition()*(1.0 - k2) + path[1].getPosition()*k2));
	int i = 1;
	for (; i<arcSize; i++)
	{
		lineRotation.push_back(std::pair<Rotation3D<double>, Rotation3D<double> >(
				Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i-1].second),
				Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i].first)));
		k1 = modifiedRatio[i - 1].second;
		k2 = modifiedRatio[i].first;
		linePosition.push_back(std::pair<Vector3D<double>, Vector3D<double> >(
				path[i].getPosition()*(1.0 - k1) + path[i + 1].getPosition()*k1,
				path[i].getPosition()*(1.0 - k2) + path[i + 1].getPosition()*k2));
	}
	lineRotation.push_back(std::pair<const Rotation3D<double>, const Rotation3D<double> >(
			Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i-1].second),
			path[i+1].getRotation()));
	k1 = modifiedRatio[i - 1].second;
//	cout << "k1 = " << k1 << endl;
	k2 = 1;
	linePosition.push_back(std::pair<Vector3D<double>, Vector3D<double> >(
			path[i].getPosition()*(1.0 - k1) + path[i + 1].getPosition()*k1,
			path[i].getPosition()*(1.0 - k2) + path[i + 1].getPosition()*k2));
	/**> 记录圆弧中间点位置和姿态 n-2 */
	for (int i=0; i<arcSize; i++)
	{
		Vector3D<double> A = linePosition[i].second; /** 圆弧左边点 */
		Vector3D<double> B = linePosition[i + 1].first; /** 圆弧右边点 */
		Vector3D<double> O = path[i + 1].getPosition(); /** 两线段交点 */
		Vector3D<double> C = (A + B)*0.5; /** 圆弧弦中点 */
		/** 求二分之一夹角 */
		Vector3D<double> OA = A - O;
		Vector3D<double> OC = C - O;
		double ct = OC.getLength()/OA.getLength();
		double theta = acos(ct);
		double k = (1.0 - sin(theta))/(ct*ct);
		/** 记录圆弧中点 */
		arcMidPosition.push_back(OC*k + O);
		/** 记录圆弧中点姿态 */
		arcMidRotation.push_back(Quaternion::interpolate(lineRotation[i].second, lineRotation[i + 1].first, 0.5));
	}
	bool indexOnLine = true;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		if (indexOnLine)
		{
			vector<HTransform3D<double> > lineEndPoint;
			HTransform3D<double> endTran = HTransform3D<double>(linePosition[i/2].second, lineRotation[i/2].second);
			lineEndPoint.push_back(endTran);
			_task.push_back(lineEndPoint);
		}
		else
		{
			vector<HTransform3D<double> > arcPoints;
			HTransform3D<double> midTran = HTransform3D<double>(arcMidPosition[(i - 1)/2], arcMidRotation[(i - 1)/2]);
			HTransform3D<double> endTran = HTransform3D<double>(linePosition[(i + 1)/2].first, lineRotation[(i + 1)/2].first);
			arcPoints.push_back(midTran);
			arcPoints.push_back(endTran);
			_task.push_back(arcPoints);
		}
		indexOnLine = !indexOnLine;
	}
}

MLABTrajectory::ptr MultiLineArcBlendPlanner::query()
{
	/**> 生成各段的trajectory, 保存统一的位置插补器和统一的姿态插补器(长度为索引) */
	vector<Trajectory::ptr> vTrajectory;
	auto posIpr = std::make_shared<SequenceInterpolator<Vector3D<double> > >();
	auto rotIpr = std::make_shared<SequenceInterpolator<Rotation3D<double> > >();

	bool isLine = _startFromLine;
	HTransform3D<double> startTran = _serialLink->getEndTransform(_qStop);
	for (auto task : _task)
	{
		if (isLine)
		{
			HTransform3D<double> endTran = task[0];
			auto linePosIpr = std::make_shared<LinearInterpolator<Vector3D<double> > >(
					startTran.getPosition(),
					endTran.getPosition(),
					Vector3D<double>::distance(startTran.getPosition(),endTran.getPosition())
					);
			auto lineRotIpr = std::make_shared<LinearInterpolator<Rotation3D<double> > >(
					startTran.getRotation(),
					endTran.getRotation(),
					linePosIpr->duration()
					);
			posIpr->addInterpolator(linePosIpr);
			rotIpr->addInterpolator(lineRotIpr);
			Trajectory::ptr trajectory = std::make_shared<Trajectory>(
					std::make_pair(linePosIpr, lineRotIpr),
					_ikSolver,
					_config);
			vTrajectory.push_back(trajectory);
			startTran = endTran;
			isLine = !isLine;
		}
		else
		{
			HTransform3D<double> midTran = task[0];
			HTransform3D<double> endTran = task[1];
			auto cirPosIpr = std::make_shared<CircularInterpolator<Vector3D<double> > >(
					startTran.getPosition(),
					midTran.getPosition(),
					endTran.getPosition());
			auto cirRotIpr = std::make_shared<LinearInterpolator<Rotation3D<double> > >(
					startTran.getRotation(),
					endTran.getRotation(),
					cirPosIpr->duration());
			Trajectory::ptr trajectory = std::make_shared<Trajectory>(
					std::make_pair(cirPosIpr, cirRotIpr),
					_ikSolver,
					_config);
			posIpr->addInterpolator(cirPosIpr);
			rotIpr->addInterpolator(cirRotIpr);
			vTrajectory.push_back(trajectory);
			startTran = endTran;
			isLine = !isLine;
		}
	}

	/**> 生成lt */
	vector<SequenceInterpolator<double>::ptr> vlt = getLt(vTrajectory);

	/**> 生成qIpr */
	vector<Interpolator<Q>::ptr> vqIpr;
	for (int i=0; i<(int)vTrajectory.size(); i++)
	{
		vqIpr.push_back(CompositeInterpolator<Q>::ptr(new CompositeInterpolator<Q>(vTrajectory[i], vlt[i])));
	}

	/**> 生成统一的trajectory, lt 和 qIpr */
	auto trajectory = std::make_shared<Trajectory> (
			std::make_pair(posIpr, rotIpr),
			_ikSolver,
			_config);

	auto lt = std::make_shared<SequenceInterpolator<double> > ();
	for_each(vlt.begin(), vlt.end(), [&](SequenceInterpolator<double>::ptr ipr){lt->appendInterpolator(ipr);});

	auto qIpr = std::make_shared<SequenceInterpolator<Q> > ();
	for_each(vqIpr.begin(), vqIpr.end(), [&](Interpolator<Q>::ptr ipr){qIpr->addInterpolator(ipr);});

	auto mlabTrajectory = std::make_shared<MLABTrajectory> (vTrajectory, vlt, vqIpr, trajectory, lt, qIpr);
	_mLABTrajectory = mlabTrajectory;
	return mlabTrajectory;
}

vector<SequenceInterpolator<double>::ptr> MultiLineArcBlendPlanner::getLt(vector<Trajectory::ptr>& trajectoryIpr)
{
	/** 策略 1 **/
//	vector<Interpolator<double>::ptr> lt;
//	double sampledl = 0.01;
//	SMPlannerEx smPlanner;
//	double v1 = 0;
//	double v2 = 0;
//	for (int i=0; i<arcSize + lineSize; i++)
//	{
//		double L = trajectoryIpr[i]->duration();
//		int count = L/sampledl + 1;
//		bool stop = i < (arcSize + lineSize - 1) ? false:true;
//		/**************** 策略 ****************************/
////		double vWanted = trajectoryIpr[i]->getMaxSpeed(count, _dqLim, velocity[i/2]); // 仅受关节速度约束
//		double vWanted = trajectoryIpr[i]->getMaxSpeed(count, _dqLim, _ddqLim, velocity[i/2]); // 受解耦的关节速度, 加速度约束
//		/*************************************************/
//		v1 = v2;
//		try{
//			lt.push_back(smPlanner.query_flexible(0, L, jerk[i/2], acceleration[i/2], v1, vWanted, v2, stop));
//		}
//		catch(const char* msg)
//		{
//			println(msg);
//			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段距离不够, 无法规划! 长度: ") + to_string(L) +
//					string(" 初始速度: ") + to_string(v1) + string(" 期望速度: ") + to_string(vWanted));
//		}
//		catch(std::string& msg)
//		{
//			println(msg);
//			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段距离不够, 无法规划! 长度: ") + to_string(L) +
//					string(" 初始速度: ") + to_string(v1) + string(" 期望速度: ") + to_string(vWanted));
//		}
//		if (fabs(v2 - vWanted) > 1e-12)
//			cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << vWanted << " -> 实际速度 = " << v2 << endl;
//	}

	/** 策略2 三速度规划 关节速度与加速度限制的线速度 线加速度不约束**/
	vector<SequenceInterpolator<double>::ptr> lt;
	SMPlannerEx smPlanner;
	vector<double> maxSpeed;
	for (int i=0; i<(int)_task.size(); i++)
	{
		double L = trajectoryIpr[i]->duration();
		int count = L/_dl + 1;
		count = (count < _countMin)? _countMin : count;
		double tempMaxSpeed = trajectoryIpr[i]->getMaxSpeed(count, _dqLim, _ddqLim, _velocity[i]);
		if (tempMaxSpeed <= 0)
			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段限制速度为0"));
		maxSpeed.push_back(tempMaxSpeed);// 受解耦的关节速度, 加速度约束
	}
	double v1 = 0;
	double v2 = 0;
	double v3 = 0;
	bool isLine = _startFromLine;
	int num = (int)trajectoryIpr.size();
	for (int i=0; i<num; i++)
	{
		double L = trajectoryIpr[i]->duration();
		try{
			/**> 终止段规划 */
			if (i == num - 1)
			{
				v1 = v3;
				v3 = maxSpeed[i];
				lt.push_back(smPlanner.query_flexible(0, L, _jerk[i], _acceleration[i], v1, v3, v3, true));
				if (fabs(v3 - maxSpeed[i]) > 1e-12)
					cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << maxSpeed[i] << " -> 实际速度 = " << v3 << endl;
			}
			/**> 除终止段外的线段 */
			else if (isLine)
			{
				v1 = v3;
				v2 = maxSpeed[i];
				v3 = maxSpeed[i + 1];
				lt.push_back(smPlanner.query_flexible(0, L, _jerk[i], _acceleration[i], v1, v2, v3, v2, v3));
				if (fabs(v2 - maxSpeed[i]) > 1e-12)
					cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << maxSpeed[i] << " -> 实际速度 = " << v2 << endl;
			}
			/**> 圆弧段 */
			else
			{
				v1 = v3;
				v3 = maxSpeed[i];
				lt.push_back(smPlanner.query_flexible(0, L, _jerk[i], _acceleration[i], v1, v3, v3, false));
				if (fabs(v3 - maxSpeed[i]) > 1e-12)
					cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << maxSpeed[i] << " -> 实际速度 = " << v3 << endl;
			}
		}
		catch(const char* msg)
		{
			println(msg);
			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段距离不够, 无法规划! 长度: ") + to_string(L) +
					string(" 初始速度: ") + to_string(v1) + string(" 期望速度: ") + to_string(v3));
		}
		catch(std::string& msg)
		{
			println(msg);
			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段距离不够, 无法规划! 长度: ") + to_string(L) +
					string(" 初始速度: ") + to_string(v1) + string(" 期望速度: ") + to_string(v3));
		}
	}
	return lt;
}

void MultiLineArcBlendPlanner::doQuery()
{
	query();
}

bool MultiLineArcBlendPlanner::stop(double t, Interpolator<Q>::ptr& stopIpr)
{
	if (_mLABTrajectory.get() == NULL)
	{
		throw( "警告<MulriLineArcBlendPlanner>: 尚未进行规划!\n");
	}
	double s0 = _mLABTrajectory->l(t);
	double v0 = _mLABTrajectory->dl(t);
	double a0 = _mLABTrajectory->ddl(t);
	double S = _mLABTrajectory->l(_mLABTrajectory->duration());
	double remainLength = S - s0;
	SMPlannerEx planner;
	int index = _mLABTrajectory->getIndexFromTime(t);
	double h = _jerk[index];
	double aMax = _acceleration[index];
	Interpolator<double>::ptr stopLt = planner.query_stop(s0, v0, a0, h, aMax); //在圆弧上减速是很危险的
	/**> 判断剩余距离是否足够停止 */
	if ((stopLt->end() - s0) >= remainLength)
	{
		cout << stopLt->end() << " " << remainLength << endl;
		cout << "错误<MulriLineArcBlendPlanner>: 距离不够, 无法停止!\n";
		return false;
	}
	Trajectory::ptr originalTrajectory = _mLABTrajectory->getTrajectory();
	stopIpr = std::make_shared<CompositeInterpolator<Q> > (originalTrajectory, stopLt);
	double s1 = stopLt->end(); //停止点的距离
	index = _mLABTrajectory->getIndexFromLength(s1); //停止点落在哪条轨迹上
	_qStop = stopIpr->end(); //记录停止点
	int count = index;
	while(count > 0)
	{
		_task.erase(_task.begin());
		_velocity.erase(_velocity.begin());
		_acceleration.erase(_acceleration.begin());
		_jerk.erase(_jerk.begin());
		_startFromLine = !_startFromLine;
		count -= 1;
	}
	if (!_startFromLine)
	{
		cout << "暂停到圆弧上\n";
		double s2 = 0; //求圆弧段终点的距离
		vector<double> vl = _mLABTrajectory->getLengthVector();
		for (int i=0; i<index+1; i++)
		{
			s2 += vl[i];
		}
		(*_task.begin())[0] = _serialLink->getEndTransform(originalTrajectory->x((s1 + s2)/2.0)); //重新计算中间点
	}
	return true;
}

void MultiLineArcBlendPlanner::resume()
{
	query();
}

bool MultiLineArcBlendPlanner::isTrajectoryExist() const
{
	if (_mLABTrajectory.get() == NULL)
		return false;
	return true;
}

Interpolator<Q>::ptr MultiLineArcBlendPlanner::getQTrajectory() const
{
	return _mLABTrajectory;
}

MultiLineArcBlendPlanner::~MultiLineArcBlendPlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
