/*
 * MultiLineArcBlendPlanner.cpp
 *
 *  Created on: Sep 26, 2017
 *      Author: a1994846931931
 */

#include "MultiLineArcBlendPlanner.h"
# include "../math/Quaternion.h"
# include "../trajectory/CircularInterpolator.h"
# include "../trajectory/LinearInterpolator.h"

using robot::math::Quaternion;

namespace robot {
namespace pathplanner {

MultiLineArcBlendPlanner::MultiLineArcBlendPlanner() {

}

MLABTrajectory::ptr MultiLineArcBlendPlanner::query(const vector<Q>& Qpath, const vector<double>& arcRatio)
{
	using cIpr = CircularInterpolator<Vector3D<double> >;
	using lIpr = LinearInterpolator<Vector3D<double> >;
	/** 路径点 n */
	vector<const HTransform3D<double> > path; //
	/** 原线段的长度 n-1*/
	vector<const double> lineLength; //
	/** 直线段上的关键姿态 (n-1)*2 */
	vector<const std::pair<const Rotation3D<double>, const Rotation3D<double> > > lineRotation; //
	/** 直线段上的关键点 (n-1)*2 */
	vector<const std::pair<const Vector3D<double>, const Vector3D<double> > > linePosition;
	/** 圆弧中间点 n-2 */
	vector<const Vector3D<double> > arcMidPosition; //
	/** 修正插补比例因子 (n-2)*2 */
	vector<std::pair<double, double> > modifiedRatio; //
	/** 圆弧位置插补器 n-2 (长度为索引) */
	vector<cIpr::ptr> arcPosIpr;
	/** 线段位置插补器 n-1 (长度为索引) */
	vector<lIpr::ptr> linePosIpr;

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

	/**> 保存路径点 */
	for (int i=0; i<pathSize; i++)
		path.push_back(_robot->getEndTransform(Qpath[i]));

	/**> 记录线段长度 */
	for (int i=0; i<lineSize; i++)
		lineLength.push_back((path[i].getPosition() - path[i + 1].getPosition()).getLength());

	/**> 记录修正比例因子 */
	for (int i=0; i<arcSize; i++)
	{
		double l1 = lineLength[i];
		double l2 = lineLength[i+1];
		double ratiol = arcRatio[i];
		double ratior = ratiol;
		if (l1 < l2)
			ratior = ratiol*l1/l2;
		else if(l1 > l2)
			ratiol = ratior*l2/l1;
		modifiedRatio.push_back(std::pair<double, double>(1 - ratiol, ratior));
	}

	/**> 记录线段的始末位姿 */
	lineRotation.push_back(std::pair<const Rotation3D<double>, const Rotation3D<double> >(
			path[0].getRotation(),
			Quaternion::interpolate(path[0].getRotation(), path[1].getRotation(), modifiedRatio[0].first)));
	int k1 = 0;
	int k2 = modifiedRatio[0].first;
	linePosition.push_back(std::pair<const Vector3D<double>, const Vector3D<double> >(
			path[0].getPosition(),
			path[0].getPosition()*(1 - k2) + path[1].getPosition()*k2));
	int i = 1;
	for (; i<arcSize; i++)
	{
		lineRotation.push_back(std::pair<const Rotation3D<double>, const Rotation3D<double> >(
				Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i-1].second),
				Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i].first)));
		k1 = modifiedRatio[i - 1].second;
		k2 = modifiedRatio[i].first;
		linePosition.push_back(std::pair<const Vector3D<double>, const Vector3D<double> >(
				path[i].getPosition()*(1 - k1) + path[i + 1].getPosition()*k1,
				path[i].getPosition()*(1 - k2) + path[i + 1].getPosition()*k2));
	}
	lineRotation.push_back(std::pair<const Rotation3D<double>, const Rotation3D<double> >(
			Quaternion::interpolate(path[i].getRotation(), path[i+1].getRotation(), modifiedRatio[i-1].second),
			path[i+1].getRotation()));
	k1 = modifiedRatio[i - 1].second;
	k2 = 1;
	linePosition.push_back(std::pair<const Vector3D<double>, const Vector3D<double> >(
			path[i].getPosition()*(1 - k1) + path[i + 1].getPosition()*k1,
			path[i].getPosition()*(1 - k2) + path[i + 1].getPosition()*k2));
	/**> 记录圆弧中间点位置 n-2 */
	for (int i=0; i<arcSize; i++)
	{
		Vector3D<double> A = linePosition[i].second; /** 圆弧左边点 */
		Vector3D<double> B = linePosition[i + 1].first; /** 圆弧右边点 */
		Vector3D<double> O = path[i + 1]; /** 两线段交点 */
		Vector3D<double> C = (A + B)*0.5; /** 圆弧弦中点 */
		/** 求二分之一夹角 */
		Vector3D<double> OA = A - O;
		Vector3D<double> OC = C - O;
		double ct = OC.getLength()/OA.getLength();
		double theta = acos(ct);
		int k = (1 - sin(theta))/(ct*ct);
		/** 记录圆弧中点 */
		arcMidPosition.push_back(OC*k + O);
	}
	for (int i=0; i<arcSize; i++)
	{
		cIpr::ptr circularIpr( new cIpr(linePosition[i].second, arcMidPosition[i], linePosition[i + 1].first));
		arcPosIpr.push_back(circularIpr);
	}
	for (int i=0; i<lineSize; i++)
	{
		lIpr::ptr lineIpr( new lIpr(linePosition[i].first, linePosition[i].second, (linePosition[i].first - linePosition[i].second).getLength()));
		linePosIpr.push_back(lineIpr);
	}
}

MultiLineArcBlendPlanner::~MultiLineArcBlendPlanner() {
	// TODO Auto-generated destructor stub
}

} /* namespace pathplanner */
} /* namespace robot */
