/*
 * LinePath.h
 *
 *  Created on: Sep 20, 2017
 *      Author: a1994846931931
 */

#ifndef LINEPATH_H_
#define LINEPATH_H_

# include "ConvertedInterpolator.h"
# include "CompositeInterpolator.h"
# include "Trajectory.h"
# include <memory>

using namespace robot::trajectory;

namespace robot {
namespace trajectory {

/**
 * @brief 直线运动的Q插补器
 */
class LinePath : public Interpolator<Q>{
public:
	using ptr = std::shared_ptr<LinePath>;

	/**
	 * @brief 构造函数
	 * @param origin [in] 位姿插补器, 由一个Vector3D<double>类型的插补器和Rotation3D<double>类型的插补器构成
	 * @param iksolver [in] 用于逆解的逆解器
	 * @param config [in] 用于逆解的位姿参数
	 * @param mappedlt [in] 对应的长度-时间插补器
	 * @param mappedtt [in] 对应的角度-时间插补器
	 */
//	LineTrajectory(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
//			std::shared_ptr<robot::ik::IKSolver> iksolver,
//			robot::model::Config config,
//			LinearCompositeInterpolator<double>::ptr mappedlt,
//			LinearCompositeInterpolator<double>::ptr mappedtt);

	LinePath(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config,
			SequenceInterpolator<double>::ptr lt,
			Trajectory::ptr trajectory);

	/** @brief 获取该规划的长度-时间插补器 */
	inline const Interpolator<double>::ptr getLIpr() const{ return _lt;}

//	/** @brief 获取该规划的角度-时间插补器 */
//	inline const Interpolator<double>::ptr getTIpr() const{ return _tt;}
	Q x(double t) const;
	Q dx(double t) const;
	Q ddx(double t) const;
	double l(double t) const;
	double dl(double t) const;
	double ddl(double t) const;
	double duration() const;

//	inline const double length(double t) const{ return _lt->x(t);}
//
//	inline const double dlength(double t) const{ return _lt->dx(t);}
//
//	inline const double ddlength(double t) const{ return _lt->ddx(t);}
//
//	inline const double theta(double t) const{ return _tt->x(t);}
//
//	inline const double dtheta(double t) const{ return _tt->dx(t);}
//
//	inline const double ddtheta(double t) const{ return _tt->ddx(t);}

//	inline const Interpolator<Vector3D<double> >::ptr getPosTIpr() const{ return _posInterpolator;}
//
//	inline const Interpolator<Rotation3D<double> >:: ptr getRotTIpr() const{ return _rotInterpolator;}

	/**
	 * @brief 获取长度l处所对应的插补器时间
	 * @param length [in] l
	 * @return 若长度小于0, 则返回0; 若长度大于最大长度, 则返回最长时间T
	 * @warning 只可用于长度-时间插补器为单调递增函数的情况下
	 */
	double timeAt(double length) const;

	/**
	 * @brief 分析直线规划的长度-时间信息
	 * @warning 由于直线规划器的时长可能因lt, tt的变化而变化, 因此只有当确信
	 * 直线插补器的时长不会改变了才能调用此函数
	 */
	void doLengthAnalysis();
	virtual ~LinePath(){}
private:
	ikInterpolator::ptr _qIpr;
	Trajectory::ptr _trajectory;

	/** @brief 直线距离-时间插补器 */
	SequenceInterpolator<double>::ptr _lt;

//	/** @brief 直线角度-时间插补器 */
//	Interpolator<double>::ptr _tt;

	/** @brief 路径长度采样
	 *
	 * 保存方式为(时刻, 对应时刻上路径长度) */
	std::vector<std::pair<double, double> > _lengthPath;

	const int _pathSize;

	/* 基类包含的成员变量
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;
	Interpolator<Vector3D<double> >::ptr _posInterpolator;
	Interpolator<Rotation3D<double> >::ptr _rotInterpolator;
	robot::model::Config _config;
	 */
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* LINETRAJECTORY_H_ */
