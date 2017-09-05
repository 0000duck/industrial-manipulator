/**
 * @brief State类
 * @date Aug 17, 2017
 * @author a1994846931931
 */

#ifndef STATE_H_
#define STATE_H_

# include "../math/Q.h"

namespace robot {
namespace kinematic {

/** @addtogroup kinematics
 * @{
 */

/**
 * @brief 描述机器人状态的类
 *
 * 分别保存每个关节的位置, 速度和加速度信息
 */
class State {
public:
	/**
	 * @brief 构造函数
	 * @param size [in] 关节的个数
	 *
	 * 全部数据会被初始化为0
	 */
	State(int size);

	/**
	 * @brief 获取所有位置
	 * @return
	 */
	const robot::math::Q& getAngle() const;

	/**
	 * @brief 获取所有速度值
	 * @return
	 */
	const robot::math::Q& getVelocity() const;

	/**
	 * @brief 获取所有加速度值
	 * @return
	 */
	const robot::math::Q& getAcceleration() const;

	/**
	 * @brief 获取索引对应关节的位置
	 * @param jointNumber [in] 索引位置
	 * @return
	 */
	const double getAngle(int jointNumber) const;

	/**
	 * @brief 获取索引对应关节的速度值
	 * @param jointNumber [in] 索引位置
	 * @return
	 */
	const double getVelocity(int jointNumber) const;

	/**
	 * @brief 获取索引对应关节的加速度值
	 * @param jointNumber [in] 索引位置
	 * @return
	 */
	const double getAcceleration(int jointNumber) const;

    /**
     * @brief 设置记录的位置
     * @param Q [in] 位置
     */
	void setAngle(const robot::math::Q&);

    /**
     * @brief 设置记录的速度值
     * @param Q [in] 速度值
     */
	void setVelocity(const robot::math::Q&);

    /**
     * @brief 设置记录的加速度值
     * @param Q [in] 加速度值
     */
	void setAcceleration(const robot::math::Q&);

    /**
     * @brief 设定索引对应关节的位置
     * @param angle [in] 位置
     * @param jointNumber [in] 索引值
     */
	void setAngle(double angle, int jointNumber);

	/**
	 * @brief 设定索引对应关节的速度值
	 * @param velocity [in] 速度
	 * @param jointNumber [in] 索引值
	 */
	void setVelocity(double velocity, int jointNumber);

	/**
	 * @brief 设定索引对应关节的加速度值
	 * @param acceleration [in] 加速度
	 * @param jointNumber [in] 索引值
	 */
	void setAcceleration(double acceleration, int jointNumber);
	~State();
private:
	/**
	 * @brief 机器人关节个数
	 */
	int _size;

	/**
	 * @brief 关节位置信息
	 */
	robot::math::Q _jointAngle;

	/**
	 * @brief 关节速度信息
	 */
	robot::math::Q _jointVelocity;

	/**
	 * @brief 关节加速度信息
	 */
	robot::math::Q _jointAccleration;
};

/** @} */
} /* namespace kinematic */
} /* namespace robot */

#endif /* STATE_H_ */
