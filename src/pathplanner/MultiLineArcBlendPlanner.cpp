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

using robot::math::Quaternion;
using robot::model::Config;
using namespace robot::common;

namespace robot {
namespace pathplanner {

MultiLineArcBlendPlanner::MultiLineArcBlendPlanner(Q qMin, Q qMax, Q dqLim, Q ddqLim,
		std::shared_ptr<robot::ik::IKSolver> ikSolver, robot::model::SerialLink* serialLink) :
	_ikSolver(ikSolver), _qMin(qMin), _qMax(qMax), _dqLim(dqLim), _ddqLim(ddqLim), _serialLink(serialLink)
{
	_size = _serialLink->getDOF();
	if (qMin.size() != _size || qMax.size() != _size || dqLim.size() != _size || ddqLim.size() != _size)
		throw ("错误<多线段圆弧混合规划器>:　构造参数中数组的长度不一致！");
}

MLABTrajectory::ptr MultiLineArcBlendPlanner::query(const vector<Q>& Qpath, const vector<double>& arcRatio, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk)
{
	using cIpr = CircularInterpolator<Vector3D<double> >;
	using lIpr = LinearInterpolator<Vector3D<double> >;
	using rIpr = LinearInterpolator<Rotation3D<double> >;

	vector<HTransform3D<double> > path; /** 路径点 n */
	vector<double> lineLength; /** 原线段的长度 n-1*/
	vector<std::pair<Rotation3D<double>, Rotation3D<double> > > lineRotation; /** 直线段上的关键姿态 (n-1)*2 */
	vector<std::pair<Vector3D<double>, Vector3D<double> > > linePosition; /** 直线段上的关键点 (n-1)*2 */
	vector<Vector3D<double> > arcMidPosition; /** 圆弧中间点 n-2 */
	vector<std::pair<double, double> > modifiedRatio; /** 修正插补比例因子 (n-2)*2 */

	/** 圆弧位置插补器 n-2 (长度为索引) */
	vector<cIpr::ptr> arcPosIpr;
	/** 线段位置插补器 n-1 (长度为索引) */
	vector<lIpr::ptr> linePosIpr;
	/** 圆弧姿态插补器 n-2 (长度为索引) */
	vector<rIpr::ptr> arcRotIpr;
	/** 线段姿态插补器 n-1 (长度为索引) */
	vector<rIpr::ptr> lineRotIpr;
	/** 各段的长度 2n-3 */
	vector<double> length;
	/** @brief 各段的ikInterpolator 2n-1 长度为索引*/
	vector<Trajectory::ptr> trajectoryIpr;
	/** @brief 各段的qInterpolator 2n-1 时间为索引*/
	vector<Interpolator<Q>::ptr > qIpr;
	/** @brief 各段的lt 2n-1 */
//	vector<Interpolator<double>::ptr> lt;
	/**> 统一的位置插补器 */
	SequenceInterpolator<Vector3D<double> >::ptr posIpr(new SequenceInterpolator<Vector3D<double> >());
	/**> 统一的姿态插补器 */
	SequenceInterpolator<Rotation3D<double> >::ptr rotIpr(new SequenceInterpolator<Rotation3D<double> >);

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
	Config config = _serialLink->getConfig(Qpath[0]);
	for (int i=1; i<pathSize; i++)
		if (config != _serialLink->getConfig(Qpath[i]))
		{
			cout << "初始Config: \n" ;
			config.print();
			cout << "该关节Config: \n" ;
			(_serialLink->getConfig(Qpath[i])).print();
			throw (std::string("错误<MultiLineArcBlendPlanner>: 第 ") + to_string(i + 1) + std::string(" 个关节的的Config和初始点不同!"));
		}

	/**> 保存路径点 */
	for (int i=0; i<pathSize; i++)
		path.push_back(_serialLink->getEndTransform(Qpath[i]));

	/**> 记录线段长度 */
	for (int i=0; i<lineSize; i++)
		lineLength.push_back((path[i].getPosition() - path[i + 1].getPosition()).getLength());

	/**> 记录修正比例因子 */
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
	/**> 记录圆弧中间点位置 n-2 */
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
		////////////////////////////////////////////////////////////////////////
		cout<<"theta = " << theta << endl;


		double k = (1.0 - sin(theta))/(ct*ct);
		/** 记录圆弧中点 */
		arcMidPosition.push_back(OC*k + O);
	}
	/**> 构造圆弧位置和姿态插补器 长度为索引*/
	for (int i=0; i<arcSize; i++)
	{
		cIpr::ptr circularIpr( new cIpr(linePosition[i].second, arcMidPosition[i], linePosition[i + 1].first));
		arcPosIpr.push_back(circularIpr);
		rIpr::ptr cRotIpr( new rIpr(lineRotation[i].second, lineRotation[i + 1].first, arcPosIpr[i]->duration()));
		arcRotIpr.push_back(cRotIpr);
	}
	/**> 构造直线位置和姿态插补器 长度为索引*/
	for (int i=0; i<lineSize; i++)
	{
		lIpr::ptr lineIpr( new lIpr(linePosition[i].first, linePosition[i].second, (linePosition[i].first - linePosition[i].second).getLength()));
		linePosIpr.push_back(lineIpr);
		rIpr::ptr lRotIpr( new rIpr(lineRotation[i].first, lineRotation[i].second, linePosIpr[i]->duration()));
		lineRotIpr.push_back(lRotIpr);
	}
	/**> 记录长度 */
	println("MultiLine: 记录长度");
	bool indexOnLine = true;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		if (indexOnLine)
		{
			length.push_back(linePosIpr[i/2]->duration());
		}
		else
		{
			length.push_back(arcPosIpr[(i - 1)/2]->duration());
		}
		indexOnLine = !indexOnLine;
	}
	println("MultiLine: 生成ikInterpolator");
	/**> 生成trajectory */
	indexOnLine = true;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		if (indexOnLine)
		{
			trajectoryIpr.push_back(Trajectory::ptr(new Trajectory(std::make_pair(linePosIpr[i/2], lineRotIpr[i/2]), _ikSolver, config)));
		}
		else
		{
			trajectoryIpr.push_back(Trajectory::ptr(new Trajectory(std::make_pair(arcPosIpr[(i - 1)/2], arcRotIpr[(i - 1)/2]), _ikSolver, config)));
		}
		indexOnLine = !indexOnLine;
	}

	/**> 生成lt */
	vector<SequenceInterpolator<double>::ptr> lt = getLt(arcSize, lineSize, trajectoryIpr, velocity, acceleration, jerk);

	/**> 生成qIpr */
	for (int i=0; i<arcSize + lineSize; i++)
	{
		qIpr.push_back(CompositeInterpolator<Q>::ptr(new CompositeInterpolator<Q>(trajectoryIpr[i], lt[i])));
	}

	/**> 生成统一的位置插补器 */
	indexOnLine = true;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		if (indexOnLine)
		{
			Interpolator<Vector3D<double> >::ptr temp = linePosIpr[i/2];
			posIpr->addInterpolator(temp);
		}
		else
		{
			posIpr->addInterpolator(arcPosIpr[(i - 1)/2]);
		}
		indexOnLine = !indexOnLine;
	}

	/**> 生成统一的姿态插补器 */
	indexOnLine = true;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		if (indexOnLine)
		{
			rotIpr->addInterpolator(lineRotIpr[i/2]);
		}
		else
		{
			rotIpr->addInterpolator(arcRotIpr[(i - 1)/2]);
		}
		indexOnLine = !indexOnLine;
	}

//	println("MultiLine: 测试");
//	/**** 测试用 ****/
//	double duration = 0;
//	for (int i=0; i<arcSize + lineSize; i++)
//	{
//		duration=trajectoryIpr[i]->duration();
//		cout << "length = " << duration << endl;
//		lt.push_back(Interpolator<double>::ptr(new LinearInterpolator<double>(0, duration, duration)));
//	}
//	for (int i=0; i<arcSize + lineSize; i++)
//	{
//		qIpr.push_back(CompositeInterpolator<Q>::ptr(new CompositeInterpolator<Q>(trajectoryIpr[i], lt[i])));
//	}

	MLABTrajectory::ptr mlabTrajectory(new MLABTrajectory(arcPosIpr, linePosIpr, arcRotIpr, lineRotIpr, length, qIpr, trajectoryIpr, lt,
			std::make_pair(posIpr, rotIpr), _ikSolver, config));
	return mlabTrajectory;
}

vector<SequenceInterpolator<double>::ptr> MultiLineArcBlendPlanner::getLt(double arcSize, double lineSize, vector<Trajectory::ptr>& trajectoryIpr, vector<double>& velocity, vector<double>& acceleration, vector<double>& jerk)
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
	double sampledl = 0.01;
	SMPlannerEx smPlanner;
	vector<double> maxSpeed;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		double L = trajectoryIpr[i]->duration();
		int count = L/sampledl + 1;
		double tempMaxSpeed = trajectoryIpr[i]->getMaxSpeed(count, _dqLim, _ddqLim, velocity[i/2]);
		if (tempMaxSpeed <= 0)
			throw(string("错误<MulriLineArcBlendPlanner>: 第") + to_string(i + 1) + string("段限制速度为0"));
		maxSpeed.push_back(tempMaxSpeed);// 受解耦的关节速度, 加速度约束
	}
	double v1 = 0;
	double v2 = 0;
	double v3 = 0;
	for (int i=0; i<arcSize + lineSize; i++)
	{
		double L = trajectoryIpr[i]->duration();
		try{
			/**> 终止段规划 */
			if (i == arcSize + lineSize - 1)
			{
				v1 = v3;
				v3 = maxSpeed[i];
				lt.push_back(smPlanner.query_flexible(0, L, jerk[i/2], acceleration[i/2], v1, v3, v3, true));
				if (fabs(v3 - maxSpeed[i]) > 1e-12)
					cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << maxSpeed[i] << " -> 实际速度 = " << v3 << endl;
			}
			/**> 除终止段外的线段 */
			else if (i%2 == 0)
			{
				v1 = v3;
				v2 = maxSpeed[i];
				v3 = maxSpeed[i + 1];
				lt.push_back(smPlanner.query_flexible(0, L, jerk[i/2], acceleration[i/2], v1, v2, v3, v2, v3));
				if (fabs(v2 - maxSpeed[i]) > 1e-12)
					cout << "MulriLineArcBlendPlanner: 第" << i + 1 << "段速度无法达到. 期望速度 = " << maxSpeed[i] << " -> 实际速度 = " << v2 << endl;
			}
			/**> 圆弧段 */
			else
			{
				v1 = v3;
				v3 = maxSpeed[i];
				lt.push_back(smPlanner.query_flexible(0, L, jerk[i/2], acceleration[i/2], v1, v3, v3, false));
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

MultiLineArcBlendPlanner::~MultiLineArcBlendPlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
