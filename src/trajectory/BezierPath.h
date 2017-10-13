/*
 * BezierPath.h
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#ifndef BEZIERPATH_H_
#define BEZIERPATH_H_

# include "BezierInterpolator.h"

namespace robot {
namespace trajectory {

/**
 * @brief 带有长度信心的贝塞尔线段路径插补器
 *
 * 长度信息在构造的时候采样获得, 采样的点数由_dl和_countMin共同确定.
 * 注意的是, 除非特别要求, 一般仅用于做速度分析的贝塞尔线段的长度信息对精度的要求
 * 不是很高. 采样点数的多少影响查找速度(查找速度关于采样点数是对数复杂度).
 */
class BezierPath : public Interpolator<Vector3D<double> >{
public:
	using ptr = std::shared_ptr<BezierPath>;
	using point = Vector3D<double>;

	/**
	 * @brief 二次贝塞尔路径构造函数
	 * @param a [in] 第一个点
	 * @param b [in] 第二个点
	 * @param c [in] 第三个点
	 * @param dl [in] 分析路径长度的精度, 会影响查找速度(参考BezierPath说明)
	 */
	BezierPath(point &a, point &b, point &c, double dl=0.01);
	BezierPath(point &a, point &b, point &c, point &d, double dl=0.01);
	BezierPath(vector<point>& pointList, double dl=0.01);
	BezierPath(BezierInterpolator::ptr bIpr, double dl=0.01);
	Vector3D<double> x(double l) const;
	Vector3D<double> dx(double l) const;
	Vector3D<double> ddx(double l) const;
	double duration() const;
	double t(double l) const;
	virtual ~BezierPath();
protected:
	/** @brief 保存的插补器 */
	BezierInterpolator::ptr _bIpr;

	/** @brief 采样精度 */
	double _dl;

	/** @brief 最少索引点数 */
	const double _countMin = 10;

	/** @brief 采样索引, 最后一个数值必与_bIpr->duration()一致 */
	vector<double> _t;

	/** @brief 采样长度, 对应_t时刻上的线段长度, 最后一个数值为本插补器的duration */
	vector<double> _length;
private:
	void init();
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* BEZIERPATH_H_ */
