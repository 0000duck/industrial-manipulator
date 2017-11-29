/**
 * @brief SmoothMotionPlanner类
 * @date Sep 7, 2017
 * @author a1994846931931
 */

#ifndef SMOOTHMOTIONPLANNER_H_
#define SMOOTHMOTIONPLANNER_H_

# include "../trajectory/Interpolator.h"
# include <vector>
# include "../trajectory/SequenceInterpolator.h"

namespace robot {
namespace pathplanner {

/** @addtogroup pathplanner
 * @brief 路径规划器
 *
 * 主要包括的类有:
 * 1. QtoQPlanner: MoveJ点对点路径规划
 * 2. SmoothMotionPlanner: 平滑路径规划器
 * 3. LinePlanner: MoveL直线规划器
 * 4. CircularPlanner: MoveC圆弧规划器
 * 5. MultiLineArcBlendPlanner: 直线段圆弧混合规划器
 * 6. JoggingPlanner:
 * @{
 */

/**
 * @brief 平滑路径规划器
 *
 * 指定给定的运动距离, 加加速度@f$ h @f$, 最大加速度@f$ a_{max} @f$ 和最大速度@f$ v_{max} @f$,
 * 自动规划出平滑的路径(指加速度和速度均以0开始和结束). 实际可能出现四种规划情况:
 *
 * <center>
 * 加速度 | 速度 | 规划方法
 * ------ | ------- | -----
 * 不能达到@f$ a_{max} @f$ | 也不能达到@f$ v_{max} @f$ | 四段式S型速度规划
 * 不能达到@f$ a_{max} @f$ | 但是可以达到@f$ v_{max} @f$ | 五段式S型速度规划
 * 能达到@f$ a_{max} @f$ | 但不能达到@f$ v_{max} @f$ | 六段式S型速度规划
 * 能达到@f$ a_{max} @f$ | 也可以达到@f$ v_{max} @f$ | 七段式S型速度规划
 * </center>
 * 参考如下:
 * @image html "./plot/smoothMotionInterpolator/4.png" "四段规划"
 * @image html "./plot/smoothMotionInterpolator/5.png" "五段规划"
 * @image html "./plot/smoothMotionInterpolator/6.png" "六段规划"
 * @image html "./plot/smoothMotionInterpolator/7.png" "七段规划"
 * @image latex "./plot/smoothMotionInterpolator/4.png" "四段规划" width=15cm
 * @image latex "./plot/smoothMotionInterpolator/5.png" "五段规划" width=15cm
 * @image latex "./plot/smoothMotionInterpolator/6.png" "六段规划" width=15cm
 * @image latex "./plot/smoothMotionInterpolator/7.png" "七段规划" width=15cm
 */
class SmoothMotionPlanner {
public:
	SmoothMotionPlanner(){}
	/**
	 * @brief 生成插补器
	 * @param s [in] 到达的距离@f$ s @f$
	 * @param h [in] 加加速度@f$ h @f$
	 * @param aMax [in] 最大加速度@f$ a_{max} @f$
	 * @param vMax [in] 最大速度@f$ v_{max} @f$
	 * @param start [in] 开始位置@f$ x_0 @f$
	 * @return 规划好的插补器
	 * @warning 插补器的生命周期和这个规划器对象的周期相同.
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr query(double s, double h, double aMax, double vMax, double start=0) const;

	/**
	 * @brief 四段规划方法生成插补器
	 * @param s [in] 到达的距离@f$ s @f$
	 * @param h [in] 加加速度@f$ h @f$
	 * @param aMax [in] 最大加速度@f$ a_{max} @f$
	 * @param vMax [in] 最大速度@f$ v_{max} @f$
	 * @param start [in] 开始位置@f$ x_0 @f$
	 * @return 规划好的插补器
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr fourLineMotion(double s, double h, double aMax, double vMax, double start=0) const;

	/**
	 * @brief 五段规划方法生成插补器
	 * @param s [in] 到达的距离@f$ s @f$
	 * @param h [in] 加加速度@f$ h @f$
	 * @param aMax [in] 最大加速度@f$ a_{max} @f$
	 * @param vMax [in] 最大速度@f$ v_{max} @f$
	 * @param start [in] 开始位置@f$ x_0 @f$
	 * @return 规划好的插补器
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr fiveLineMotion(double s, double h, double aMax, double vMax, double start=0) const;

	/**
	 * @brief 六段规划方法生成插补器
	 * @param s [in] 到达的距离@f$ s @f$
	 * @param h [in] 加加速度@f$ h @f$
	 * @param aMax [in] 最大加速度@f$ a_{max} @f$
	 * @param vMax [in] 最大速度@f$ v_{max} @f$
	 * @param start [in] 开始位置@f$ x_0 @f$
	 * @return 规划好的插补器
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr sixLineMotion(double s, double h, double aMax, double vMax, double start=0) const;

	/**
	 * @brief 七段规划方法生成插补器
	 * @param s [in] 到达的距离@f$ s @f$
	 * @param h [in] 加加速度@f$ h @f$
	 * @param aMax [in] 最大加速度@f$ a_{max} @f$
	 * @param vMax [in] 最大速度@f$ v_{max} @f$
	 * @param start [in] 开始位置@f$ x_0 @f$
	 * @return 规划好的插补器
	 */
	robot::trajectory::SequenceInterpolator<double>::ptr sevenLineMotion(double s, double h, double aMax, double vMax, double start=0) const;

	/**
	 * @brief 析构函数
	 *
	 * 释放规划出插补器对象占用的内存
	 */
	virtual ~SmoothMotionPlanner();
private:
	/**
	 * @brief @f$ s_1 @f$ 长度
	 *
	 * 加速度以加加速@f$ h @f$ 从0到@f$ a_{max} @f$ , 匀加速一段时间, 加速度再以加加速@f$ h @f$ 从@f$ a_{max} @f$
	 * 到0(此时认为达到@f$ v_{max} @f$ ), 所经过的总路径长度(前提是匀加速时间段大于等于0). <br>
	 *  - @f$ s_1 = \frac{h v_{max}^2 + v_{max} a_{max}^2}{2 h a+_{max}} @f$
	 */
	double s1(double h, double vMax, double aMax) const;

	/**
	 * @brief @f$ s_2 @f$ 长度
	 *
	 * 加速度以加加速@f$ h @f$ 从0到@f$ a_{max} @f$ , 加速度再以加加速@f$ h @f$ 直接从@f$ a_{max} @f$ 到0,
	 * 所经过的总路径长度. <br>
	 *  - @f$ s_2 = \frac{a_{max}^3}{h^2} @f$
	 */
	double s2(double h, double aMax) const;
private:
};

/** @} */
} /* namespace pathplanner */
} /* namespace robot */

#endif /* SMOOTHMOTIONPLANNER_H_ */
