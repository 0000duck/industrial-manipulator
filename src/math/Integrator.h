/*
 * Integrator.h
 *
 *  Created on: Sep 22, 2017
 *      Author: a1994846931931
 */

#ifndef INTEGRATOR_H_
#define INTEGRATOR_H_

# include "../trajectory/Interpolator.h"

# include <vector>

using std::vector;
using namespace robot::trajectory;

namespace robot {
namespace math {

class Integrator {
public:
	Integrator();

	/**
	 * @brief 根据t向量和位置插补器采样在各个时刻上路径到其初始点的长度
	 * @param positionIpr [in] 位置插补器
	 * @param t [in] 采样时刻向量
	 * @return 插补器采样在t的各个时刻上路径到其初始点的长度
	 * @note 长度的精度和t向量的精度(密集程度)有关. 如果只需要获取总长度, 可用
	 * double integrate(Interpolator<Vector3D<double> >::ptr positionIpr, int count);
	 * 经测试, 速度可以提高20%左右;
	 */
	vector<double> integrate(Interpolator<Vector3D<double> >::ptr positionIpr, vector<double> t);

	/**
	 * @brief 获取路径总长度
	 * @param positionIpr [in] 待分析的位置插补器(路径)
	 * @param count [in] 采样点数
	 * @return 路径从时刻0到duration()返回的时刻之间的长度
	 * @note 采样的精度和count有关; 积分方式为把采样点之间的路径看做是一段直线;
	 */
	double integrate(Interpolator<Vector3D<double> >::ptr positionIpr, int count);
	virtual ~Integrator();
};

} /* namespace math */
} /* namespace robot */

#endif /* INTEGRATOR_H_ */
