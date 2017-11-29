/**
 * @brief BezierInterpolator
 * @date Oct 12, 2017
 * @author a1994846931931
 */

#ifndef BEZIERINTERPOLATOR_H_
#define BEZIERINTERPOLATOR_H_

# include "Interpolator.h"
# include "../math/Vector3D.h"
# include "../common/printAdvance.h"

using robot::math::Vector3D;
using std::vector;

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 通用贝塞线段插补类
 * duration由const double _duration给定, 用户不能自由指定;
 *
 * 二次和三次贝塞尔曲线获取位置由公式直接计算, 速度较快. 更高次的通过循环获取, 次数越高, 循环次数越多.
 */
class BezierInterpolator : public Interpolator<Vector3D<double> > {
public:
	using point = Vector3D<double>;
	/**
	 * @brief 二次贝塞尔
	 * @param a [in] 点一
	 * @param b [in] 点二
	 * @param c [in] 点三
	 */
	BezierInterpolator(point &a, point &b, point &c);

	/**
	 * @brief 三次贝塞尔
	 * @param a [in] 点一
	 * @param b [in] 点二
	 * @param c [in] 点三
	 * @param d [in] 点四
	 */
	BezierInterpolator(point &a, point &b, point &c, point &d);

	/**
	 * @brief n次的贝塞尔线段
	 * @warning 不建议使用高次贝塞尔
	 * @param pointList [in] 点集合
	 */
	BezierInterpolator(vector<point>& pointList);

	/**
	 * @brief 获取s索引对应的位置
	 * @param s [in] s索引, 范围为0~_duration
	 * @return s索引处的位置
	 */
	point x(double s) const;

	/**
	 * @brief 获取s索引对应的速度
	 * @param s [in] s索引, 范围为0~_duration
	 * @return s索引处的速度
	 */
	point dx(double s) const;

	/**
	 * @brief 获取s索引对应的加速度
	 * @param s [in] s索引, 范围为0~_duration
	 * @return s索引处的加速度
	 */
	point ddx(double s) const;

	/**@brief 索引长度 */
	double duration() const;

	virtual ~BezierInterpolator();
protected:
	point x3(double k, const vector<point>& pointList) const;
	point dx3(double k, const vector<point>& pointList) const;
	point ddx3(double k, const vector<point>& pointList) const;
	point x4(double k, const vector<point>& pointList) const;
	point dx4(double k, const vector<point>& pointList) const;
	point ddx4(double k, const vector<point>& pointList) const;
	point xn(double k, const vector<point>& pointList) const;
	point dxn(double k, const vector<point>& pointList) const;
	point ddxn(double k, const vector<point>& pointList) const;

protected:
	/**> 返回p1*(1.0 -k) + p2*k */
	static point interpolate(const point &p1, const point &p2, const double &k);

protected:
	/**> 关键点的个数 */
	int _size;

	/**> 贝赛尔曲线关键点 */
	vector<point> _vpoint;

	/**> 归一因子, 用1.0即可 */
	const double _duration = 1.0;
};

/** @} */

} /* namespace trajectory */
} /* namespace robot */

#endif /* BEZIERINTERPOLATOR_H_ */
