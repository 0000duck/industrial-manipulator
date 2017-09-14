/**
 * @brief Interpolator类
 * @date Sep 1, 2017
 * @author a1994846931931
 */

#ifndef INTERPOLATOR_H_
#define INTERPOLATOR_H_

# include <memory>
# include "../math/Q.h"
# include "../kinematics/State.h"
# include <math.h>

using namespace robot::math;
using robot::kinematic::State;

namespace robot {
namespace trajectory {

/** @addtogroup trajectory
 * @brief 插补器, 生成平滑的插补函数, 提供x(t), dx(t), ddx(t)和duration()函数接口
 *
 * 包括的类有:
 * 1. Interpolator: 插补器基类
 * 2. LinearInterpolator: 线性插补器
 * 3. CircularInterpolator: 圆弧插补器
 * 4. PolynomialInterpolator: 多项式插补器
 * 5. SequenceInterpolator: 将多个同输出类型的插补器组合成新的插补器
 * 6. ConvertedInterpolator: 转变插补器的输出类型
 *
 * 可以通过组合, 生成复杂的路径插补器. 例如, 对于抛物线路径规划的插补器, 可以用PolynomialInterpolator3生成
 * 加速段和减速段的三次路径, 用LinearInterpolator生匀速段的路径, 再用SequenceInterpolator合并成一个
 * 插补器. 这只是简单的例子, 实际中还要考虑三维空间坐标的转换, 角度的规划, 以及把位姿通过逆运动学求解成Q,
 * 最后组合成输出为关节角Q的插补器.
 * @{
 */

/**
 * @brief 插补器接口
 *
 * 所有插补器一致的基类, 定义了公共的函数借口x(t), dx(t), ddx(t)和duration().
 * 参照具体类的实现
 */
template <class T>
class Interpolator {
public:
	using ptr = std::shared_ptr<Interpolator<T> >;
	Interpolator(){}
	virtual ~Interpolator(){}

	/**
	 * @brief t时刻的位置
	 * @param t [in] 时间t
	 * @return 位置
	 */
	virtual T x(double t) const = 0;

	/**
	 * @brief t时刻的速度
	 * @param t [in] 时间t
	 * @return 速度
	 */
	virtual T dx(double t) const = 0;

	/**
	 * @brief t时刻的加速度
	 * @param t [in] 时间t
	 * @return 加速度
	 */
	virtual T ddx(double t) const = 0;

	/**
	 * @brief 总时长
	 * @return 规定的总时长
	 */
	virtual double duration() const = 0;
};

/**
 * @brief Q插补器
 *
 * 添加了采样获取极限值的方法
 */
template <>
class Interpolator<Q>{
public:
	using ptr = std::shared_ptr<Interpolator<Q> >;
	Interpolator(){}
	virtual ~Interpolator(){}

	/**
	 * @brief t时刻的位置
	 * @param t [in] 时间t
	 * @return 位置
	 */
	virtual Q x(double t) const = 0;

	/**
	 * @brief t时刻的速度
	 * @param t [in] 时间t
	 * @return 速度
	 */
	virtual Q dx(double t) const = 0;

	/**
	 * @brief t时刻的加速度
	 * @param t [in] 时间t
	 * @return 加速度
	 */
	virtual Q ddx(double t) const = 0;

	/**
	 * @brief 总时长
	 * @return 规定的总时长
	 */
	virtual double duration() const = 0;

	/**
	 * @brief 获得State(Q, dQ, ddQ)
	 * @param t [in] 时间
	 * @param precision [in] 采样精度
	 * @return [Q, dQ, ddQ]
	 * @note 返回std::vector<Q>而不是State的原因是目前构造State的方法不合理, 需要
	 * 复制各个Q, 相比之下用std::vector<Q>更为经济.
	 *
	 * 采用数值方法求速度和加速度, 可以指定采样的精度. 由于速度需要求解两次x(t), 加速度
	 * 需要求解三次x(t), 如果用数值方法分开求解这三个数据将需要6次求x(t). 用这个方法只
	 * 需要求解3次x(t), 计算上更为节省.
	 */
	virtual State getState(double t, double precision=0.0001) const
	{
		Q x0 = this->x(t);
		Q x1 = this->x(t + precision);
		Q x2 = this->x(t + precision*2);
		Q dx = (x1 - x0)/precision;
		Q ddx = (x0 + x2 - x1*2.0)/(precision*precision);
		return State(x0, dx, ddx);
	}

	virtual std::pair<Q, Q> getLimQ(int step)
	{
		double T = this->duration();
		double dt = T/(step - 1);
		int size = (this->x(0)).size();
		Q minQ = this->x(0);
		Q maxQ = minQ;
		Q xresult;
		for (double t=0; t<=T; t+=dt)
		{
			xresult = this->dx(t);
			for (int i=0; i<size; i++)
			{
				if (xresult[i] > maxQ[i])
					maxQ(i) = xresult[i];
				else if (xresult[i] < minQ[i])
					minQ(i) = xresult[i];
			}
		}
		return std::pair<Q, Q>(minQ, maxQ);
	}

	virtual Q getMaxdQ(int step)
	{
		double T = this->duration();
		double dt = T/(step - 1);
		int size = (this->dx(0)).size();
		Q dqMax = Q::zero(size);
		Q dxresult;
		for (double t=0; t<=T; t+=dt)
		{
			dxresult = this->dx(t);
			for (int i=0; i<size; i++)
			{
				if (dqMax(i) < fabs(dxresult[i]))
					dqMax(i) = fabs(dxresult[i]);
			}
		}
		return dqMax;
	}

	virtual Q getMaxddQ(int step) const
	{
		double T = this->duration();
		double dt = T/(step - 1);
		int size = (this->ddx(0)).size();
		Q ddqMax = Q::zero(size);
		Q ddxresult;
		for (double t=0; t<=T; t+=dt)
		{
			ddxresult = this->ddx(t);
			for (int i=0; i<size; i++)
			{
				if (ddqMax(i) < fabs(ddxresult[i]))
					ddqMax(i) = fabs(ddxresult[i]);
			}
		}
		return ddqMax;
	}
};

/**
 * @brief 常量插补器
 *
 * 返回固定常亮, 速度和加速度均为0.
 */
template <class T>
class FixedInterpolator: public Interpolator<T>{
public:
	/**
	 * @brief 构造函数
	 * @param constant [in] 固定返回的常量
	 * @param duration [in] 插补时长
	 */
	FixedInterpolator(T constant, double duration): _constant(constant), _zero(constant*0), _duration(duration){}
	T x(double t) const
	{
		return _constant;
	}

	T dx(double t) const
	{
		return _zero;
	}

	T ddx(double t) const
	{
		return _zero;
	}

	double duration() const
	{
		return _duration;
	}
private:
	T _constant;
	T _zero;
	double _duration;
};

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* INTERPOLATOR_H_ */
