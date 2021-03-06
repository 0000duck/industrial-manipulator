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
# include "../model/SerialLink.h"
# include "../math/Vector3D.h"
# include <math.h>
# include "../common/printAdvance.h"

using namespace robot::math;
using robot::model::SerialLink;
using robot::kinematic::State;
using robot::math::Vector3D;
using namespace robot::common;

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
	 * @brief 返回末端位置
	 * @return 末端位置
	 */
	T end() const
	{
		return x(duration());
	}

	/**
	 * @brief 返回开始位置
	 * @return 开始位置
	 */
	T start() const
	{
		return x(0);
	}

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
	 * @brief 返回末端位置
	 * @return 末端位置
	 */
	Q end() const
	{
		return x(duration());
	}

	/**
	 * @brief 返回开始位置
	 * @return 开始位置
	 */
	Q start() const
	{
		return x(0);
	}

	/**
	 * @brief 总时长
	 * @return 规定的总时长
	 */
	virtual double duration() const = 0;

	/**
	 * @brief 获得State
	 * @param t [in] 时间
	 * @param precision [in] 采样精度
	 * @return state
	 *
	 * 采用数值方法求速度和加速度, 可以指定采样的精度. 由于速度需要求解两次x(t), 加速度
	 * 需要求解三次x(t), 如果用数值方法分开求解这三个数据将需要6次求x(t). 用这个方法只
	 * 需要求解3次x(t), 计算上更为节省.
	 */
	virtual State getState(double t, double precision=0.00001) const
	{
		Q x0 = this->x(t);
		Q x1 = this->x(t + precision);
		Q x2 = this->x(t + precision*2);
		Q dx = (x1 - x0)/precision;
		Q ddx = (x0 + x2 - x1*2.0)/(precision*precision);
		return State(x0, dx, ddx);
	}

	/**
	 * @brief 获取关节上下限
	 * @param step [in] 采样数量
	 * @return 返回关节上下限制[min, maax]
	 */
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

	/**
	 * @brief 返回关节最大速度
	 * @param step [in] 采样数量
	 * @return 各个关节的最大速度(绝对值)
	 */
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

	/**
	 * @brief 返回关节的最大加速度
	 * @param step [in] 采样数量
	 * @return 各个关节的最大加速度(绝对值)
	 */
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

	/**
	 * @brief 对x(t)函数进行采样
	 * @param step [in] 采样数量
	 * @return 在0~duration()之间采样的step个位置点
	 */
	virtual std::vector<Q> xSample(int step)
	{
		std::vector<Q> q;
		double T = this->duration();
		double dt = T/double(step - 1);
		for (double t=0; t<=T; t+=dt)
			q.push_back(this->x(t));
		return q;
	}

	/**
	 * @brief 对dx(t)函数进行采样
	 * @param step [in] 采样数量
	 * @return 在0~duration()之间采样的step个速度点
	 */
	virtual std::vector<Q> dxSample(int step)
	{
		std::vector<Q> q;
		double T = this->duration();
		double dt = T/double(step - 1);
		for (double t=0; t<=T; t+=dt)
			q.push_back(this->dx(t));
		return q;
	}

	/**
	 * @brief 对ddx(t)函数进行采样
	 * @param step [in] 采样数量
	 * @return 在0~duration()之间采样的step个加速度点
	 */
	virtual std::vector<Q> ddxSample(int step)
	{
		std::vector<Q> q;
		double T = this->duration();
		double dt = T/double(step - 1);
		for (double t=0; t<=T; t+=dt)
			q.push_back(this->ddx(t));
		return q;
	}

	/**
	 * @brief 路径长度分析
	 * @param serialink [in] 用于正运动学的机器人模型
	 *
	 * 分析路径的长度, 并保存采样结果到类内部.
	 * @note 会检查是否已经做过路径分析, 同样的路径不会做两次分析
	 */
	virtual void doLengthAnalysis(const SerialLink* serialink)
	{
		if ((int)_trajectoryLength.size() > 0)
		{
			println("路径分析仅做一次");
			return;
		}
		int step = 1000; //采样次数
		double T = this->duration();
		double dt = T/(step - 1);
		Vector3D<double> position1;
		Vector3D<double> position2 = serialink->getEndPosition(this->x(0));
		_trajectoryLength.push_back(std::pair<double, double>(0, 0));
		int i = 0;
		for (double t = dt; t<= T; t+=dt)
		{
			position1 = position2;
			position2 = serialink->getEndPosition(this->x(t));
			_trajectoryLength.push_back(std::pair<double, double>(t, _trajectoryLength[i++].second + (position2 - position1).getLength()));
		}
	}

	/**
	 * @brief 根据长度获取时刻
	 * @param length [in] 路径长度(位置)
	 * @return 从路径起点开始, 经过length长度时, 所在的时刻
	 *
	 * 由于路径长度关于时间是递增的, 因此采用二分法来进行计算.
	 */
	virtual double timeAt(double length)
	{
		if ((int)_trajectoryLength.size() <= 0)
			throw(std::string("错误<Interpolator>: 尚未进行过路径长度分析, 无法获取目标位置的时间!"));
		double t;
		int size = _trajectoryLength.size();
		if (length <= 0)
			t = 0;
		else if (length >= _trajectoryLength[size - 1].second)
			t = _trajectoryLength[size - 1].first;
		else
		{
			int a = 0;
			int b = size - 1;
			int c;
			while((b - a) > 1)
			{
				c = (int)((a + b)/2);
				if (length < _trajectoryLength[c].second)
					b = c;
				else
					a = c;
			}
			double ta = _trajectoryLength[a].first;
			double tb = _trajectoryLength[b].first;
			double ya = _trajectoryLength[a].second;
			double yb = _trajectoryLength[b].second;
			if (ya == yb)
				t = ta;
			else
				t = (tb - ta)*(length - ya)/(yb - ya) + ta;
		}
		return t;
	}
private:
	/**
	 * @brief 路径长度采样
	 *
	 * 保存方式为(时刻, 对应时刻上路径长度)
	 */
	std::vector<std::pair<double, double> > _trajectoryLength;
};

/**
 * @brief 常量插补器
 *
 * 返回固定常亮, 速度和加速度均为0.
 */
template <class T>
class FixedInterpolator: public Interpolator<T>{
public:
	using ptr = std::shared_ptr<FixedInterpolator>;

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
