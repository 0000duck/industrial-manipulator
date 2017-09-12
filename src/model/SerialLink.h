/**
 * @brief SerialLink类
 * @date Aug 15, 2017
 * @author a1994846931931
 */

#ifndef SERIALLINK_H_
#define SERIALLINK_H_

# include "../kinematics/Frame.h"
# include "../math/HTransform3D.h"
# include "Link.h"
# include "DHTable.h"
# include "DHParameters.h"
# include "../math/Q.h"
# include "../model/Jacobian.h"
# include "Jacobian.h"
# include "Config.h"
# include "../kinematics/State.h"
# include "../math/Quaternion.h"

using robot::kinematic::Frame;

namespace robot {
namespace model {

/** @addtogroup model
 * @brief 机器人模型的表达, DH参数, 雅克比矩阵等.
 *
 * 包括的类有:
 * 1. Config: 机器人姿态配置文件
 * 2. DHParameters: DH参数
 * 3. DHTable: DH参数表
 * 4. Jacobian: 雅克比矩阵
 * 5. Link: 机器人关节模型
 * 6. SerialLink: 串行机器人模型
 * @{
 */

/**
 * @brief 串行机器人类
 *
 */
class SerialLink {
public:
	/**
	 * @brief 默认构造函数
	 * @param tool [in] 工具指定的Frame偏置, 若不指定, 则按照Frame的默认构造函数构造
	 */
	SerialLink(Frame* tool=NULL);

	/**
	 * @brief 根据Link指针的数组构造
	 * @param linkList [in] Link的指针数组
	 * @param tool [in] 工具指定的Frame偏置, 若不指定, 则按照Frame的默认构造函数构造
	 */
	SerialLink(std::vector<Link*>, Frame* tool=NULL);

	/**
	 * @brief 添加关节
	 * @param link [in] 在末端添加一个关节
	 */
	void append(Link*);

	/**
	 * @brief 移除关节
	 * @return 将移除的最后一个关节的地址返回
	 */
	Link* pop();

	/**
	 * @brief 获取自由度
	 * @return 关节的个数
	 */
	int getDOF() const;

	/** @brief 获取DH参数表 */
	DHTable getDHTable() const;

	/**
	 * @brief 获取变换矩阵
	 * @param startLink [in] 开始关节的索引位置(从0开始)
	 * @param endLink [in] 结束关节的索引位置(从0开始)
	 * @param q [in] 关节数值
	 * @return 在关节数值为Q时关节startLink到endLink的变换矩阵
	 */
	HTransform3D<double> getTransform(unsigned int startLink, unsigned int endLink, const robot::math::Q& q) const;

	/**
	 * @brief 获取末端变换矩阵
	 * @return 当所有关节均为0时, 末端执行器相对于基座标的变换矩阵
	 */
	HTransform3D<double> getEndTransform(void) const;

	/**
	 * @brief 获取末端变换矩阵
	 * @return 当关节为Q时, 末端执行器相对于基座标的变换矩阵
	 */
	HTransform3D<double> getEndTransform(const robot::math::Q& q) const;

	/**
	 * @brief 获取四元数表达的旋转量
	 * @param startLink [in] 开始关节的索引位置(从0开始)
	 * @param endLink [in] 结束关节的索引位置(从0开始)
	 * @param q [in] 关节数值
	 * @return 在关节数值为Q时关节startLink到endLink的旋转量(四元数表示)
	 */
	Quaternion getQuaternion(unsigned int startLink, unsigned int endLink, const robot::math::Q& q) const;

	/**
	 * @brief 获取末端变换矩阵
	 * @return 当所有关节均为0时, 末端执行器相对于基座标的旋转量(四元数表示)
	 */
	Quaternion getEndQuaternion(void) const;

	/**
	 * @brief 获取末端变换矩阵
	 * @return 当关节为Q时, 末端执行器相对于基座标的旋转量(四元数表示)
	 */
	Quaternion getEndQuaternion(const Q& q) const;

	/**
	 * @brief 获取默认雅克比矩阵
	 * @return 当所有关节为0时机器人的雅克比矩阵
	 */
	Jacobian getJacobian() const;

	/**
	 * @brief 获取雅克比矩阵
	 * @param q [in] 关节数值
	 * @return 当关节为Q时机器人的雅克比矩阵
	 */
	Jacobian getJacobian(const robot::math::Q& q) const;

	/** @brief 获取关节数值 */
	const robot::math::Q getQ() const;

	/** @brief 设置关节数值 */
	void setQ(robot::math::Q);

	/**
	 * @brief 获取关节姿态配置
	 * @param q [in] 关节数值
	 * @return 关节数值为Q时机器人的姿态配置
	 */
	Config getConfig(const Q&) const;

	/**
	 * @brief 获取末端执行器速度
	 * @param state [in] 机器人状态
	 * @return 机器人状态为state时末端执行器的速度
	 */
	const robot::math::Q getEndVelocity(const robot::kinematic::State& state) const;

	/**
	 * @brief 获取关节速度
	 * @param endVelocity [in] 末端执行器的速度
	 * @param robotPos [in] 机器人当前的姿态
	 * @return 各个关节的速度
	 */
	const robot::math::Q getEndVelocity(const robot::math::Q endVelocity, const robot::math::Q robotPos) const;

	/** @brief 格式化打印
	 * @todo 未定义功能
	 */
	void print();
	virtual ~SerialLink();
private:
	/** @brief 关节地址列表 */
	std::vector<Link*> _linkList;

	/** @brief 基坐标系 */
	Frame _worldFrame;

	/** @brief 末端到工具中心点的偏置 */
	Frame _endToTool;

};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* SERIALLINK_H_ */
