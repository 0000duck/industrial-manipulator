/**
 * @brief ConvertedInterpolator类
 * @date: Sep 7, 2017
 * @author a1994846931931
 */

#ifndef CONVERTEDINTERPOLATOR_H_
#define CONVERTEDINTERPOLATOR_H_

# include "Interpolator.h"
# include <vector>
# include "../math/Q.h"
# include "../common/printAdvance.h"
# include <memory>
# include "../ik/IKSolver.h"
# include "../model/Config.h"
# include "../common/printAdvance.h"
# include <algorithm>

using std::vector;

namespace robot {
namespace trajectory {

/**
 * @addtogroup trajectory
 * @{
 */

/**
 * @brief 转换类型的插补器模板类
 *
 * 可以将类型B构造成输出类型为T的插补器, 类型B默认是一种插补器. 在默认情况下,
 * 尝试将B插补器的输出强制类型转换成T类的输出. 类型B也可以是别的种类, 例如一个元素为
 * 插补器指针的vector, 甚至是一个常量.
 */
template <class B, class T>
class ConvertedInterpolator: public Interpolator<T> {
public:
	using ptr = std::shared_ptr<ConvertedInterpolator<B, T> >;
	/**
	 * @brief 构造函数
	 *
	 * 记录源插补器的指针
	 * @param origin [in] 源插补器
	 */
	ConvertedInterpolator(std::shared_ptr<Interpolator<B> > origin)
	{
		_OriginalInterpolator = origin;
	}

	T x(double t) const
	{
		return T(_OriginalInterpolator->x(t));
	}

	T dx(double t) const
	{
		return T(_OriginalInterpolator->dx(t));
	}

	T ddx(double t) const
	{
		return T(_OriginalInterpolator->ddx(t));
	}

	double duration() const
	{
		return _OriginalInterpolator->duration();
	}
	virtual ~ConvertedInterpolator(){}
private:
	/** @brief 源插补器 */
	std::shared_ptr<Interpolator<B> > _OriginalInterpolator;
};

/**
 * @brief 转换自double类型插补器指针容器, 转换至Q
 */
template<>
class ConvertedInterpolator<std::vector<Interpolator<double>::ptr > , robot::math::Q>: public Interpolator<robot::math::Q> {
public:
	/**
	 * @brief 构造函数
	 * @param origin [in] 用于构造Q插补器的double插补器集合
	 */
	ConvertedInterpolator(std::vector<Interpolator<double>::ptr > origin)
	{
		_interpolatorList.assign(origin.begin(), origin.end());
		_size = _interpolatorList.size();
		if (_size <= 0)
			throw ("错误: 用于构造Q类型的插补器容器大小不能为0!");
	}

	robot::math::Q x(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->x(t);
		return q;
	}

	robot::math::Q dx(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->dx(t);
		return q;
	}

	robot::math::Q ddx(double t) const
	{
		robot::math::Q q = robot::math::Q::zero(_size);
		for (int i=0; i<_size; i++)
			q(i) = _interpolatorList[i]->ddx(t);
		return q;
	}

	double duration() const
	{
		return _interpolatorList[0]->duration();
	}
	virtual ~ConvertedInterpolator(){}
private:
	/** @brief 源插补器列表 */
	std::vector<Interpolator<double>::ptr > _interpolatorList;

	/** @brief 插补器列表大小 */
	int _size;
};

/**
 * @brief 转换位姿插补器容器到Q(使用ikSolver)
 */
template<>
class ConvertedInterpolator<std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr > , robot::math::Q>: public Interpolator<robot::math::Q> {
public:
	struct qVelAcc{
		vector<Q> dq;
		vector<Q> ddq;
	};
public:
	using ptr = std::shared_ptr<ConvertedInterpolator<std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr > , robot::math::Q> >;
	/**
	 * @brief 构造函数
	 * @param origin [in] 位姿插补器, 由一个Vector3D<double>类型的插补器和Rotation3D<double>类型的插补器构成
	 * @param iksolver [in] 用于逆解的逆解器
	 * @param config [in] 用于逆解的位姿参数
	 */
	ConvertedInterpolator(std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr >  origin,
			std::shared_ptr<robot::ik::IKSolver> iksolver,
			robot::model::Config config)
	{
		_ikSolver = iksolver;
		_posInterpolator = origin.first;
		_rotInterpolator = origin.second;
		if (fabs(_posInterpolator->duration() - _rotInterpolator->duration()) > 0.001)
			common::println("警告<ikInterpolator>: 位置插补器与姿态插补器的周期不同!");
	}

	virtual robot::math::Q x(double t) const
	{
		try{
			std::vector<robot::math::Q> result = (_ikSolver->solve(HTransform3D<double>(_posInterpolator->x(t), _rotInterpolator->x(t)), _config));
			if ((int)result.size() <= 0)
			{
				robot::common::println("无法逆解的末端位姿: ");
				HTransform3D<double>(_posInterpolator->x(t), _rotInterpolator->x(t)).print();
				throw (std::string("错误<ikInterpolator>: 无法进行逆解!"));
			}
			return result[0];
		}
		catch(std::string& msg)
		{
			throw(std::string("错误<ikInterpolator>: 无法进行逆解!\n") + msg);
		}
	}

	/** @todo 如何处理 */
	virtual robot::math::Q dx(double t) const
	{
		return (this->x(t + 0.0001) - this->x(t))*10000.0;
	}

	virtual robot::math::Q ddx(double t) const
	{
		return ((this->x(t + 0.0002)) - (this->x(t + 0.0001))*2.0 +( this->x(t)))*100000000.0;
	}

	double duration() const
	{
		return _posInterpolator->duration();
	}
	qVelAcc sampleVelAcc(const int count, double precision=0.0001)
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
	/**
	 * @brief 采样分析ikInterpolator指示路径上的速度上下限
	 * @param count [in] 采样的点数
	 * @param precision [in] 速度, 加速度数值计算的精度(dt)
	 * @return {{dqMin, dqMax}, {ddqMin, ddqMax}}
	 */
	std::pair<std::pair<Q, Q>, std::pair<Q, Q> > minmax(const int count = 100, double precision=0.0001)
	{
		qVelAcc result = this->sampleVelAcc(count, precision);
		vector<Q> dq = result.dq;
		vector<Q> ddq = result.ddq;
		Q dqMin = dq[0];
		Q dqMax = dqMin;
		Q ddqMin = ddq[0];
		Q ddqMax = ddqMin;
//		for_each(dq.begin(), dq.end(), [](Q& q){q.abs()});
//		for_each(ddq.begin(), ddq.end(), [](Q& q){q.abs()});
		for_each(dq.begin(), dq.end(), [&](Q& q){q.doMinmax(dqMin, dqMax);});
		for_each(ddq.begin(), ddq.end(), [&](Q& q){q.doMinmax(ddqMin, ddqMax);});
		return std::make_pair(std::make_pair(dqMin, dqMax), std::make_pair(ddqMin, ddqMax));
	}
	virtual ~ConvertedInterpolator(){}
protected:
	std::shared_ptr<robot::ik::IKSolver> _ikSolver;
	Interpolator<Vector3D<double> >::ptr _posInterpolator;
	Interpolator<Rotation3D<double> >::ptr _rotInterpolator;
	robot::model::Config _config;
};

using ikInterpolator = ConvertedInterpolator<std::pair<Interpolator<Vector3D<double> >::ptr , Interpolator<Rotation3D<double> >::ptr > , robot::math::Q>;

/** @} */
} /* namespace trajectory */
} /* namespace robot */

#endif /* CONVERTEDINTERPOLATOR_H_ */
