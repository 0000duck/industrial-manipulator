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
# include <memory>

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
	using ptr = std::shared_ptr<SerialLink>;

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
	SerialLink(std::vector<Link::ptr>, Frame* tool=NULL);

	/**
	 * @brief 添加关节
	 * @param link [in] 在末端添加一个关节
	 */
	void append(Link::ptr link);

	/**
	 * @brief 添加关节
	 * @param link [in] 在末端添加一个关节指针
	 * @deprecated 推荐使用shared_ptr的方法
	 */
	void append(Link* link);

	/**
	 * @brief 设置末端工具
	 * @param tool
	 */
	void setTool(Frame* tool);

	/**
	 * @brief 设置默认tool
	 */
	void setDefaultTool();

	/**
	 * @brief 移除关节
	 * @return 将移除的最后一个关节的地址返回
	 */
	Link::ptr pop();

	/** @brief 获取关节 */
	inline Link::ptr getLink(int index) const
	{
		return _linkList[index];
	}

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
	 * @brief 获取末端的位置
	 * @return 当所有关节均为0时, 末端执行器的位置
	 */
	Vector3D<double> getEndPosition(void) const;

	/**
	 * @brief 获取末端的位置
	 * @param q [in] 关节位置
	 * @return  当关节为Q时, 末端执行器的位置
	 */
	Vector3D<double> getEndPosition(const robot::math::Q& q) const;

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
	 * @brief todo 考虑tool计算雅克比矩阵
	 * @brief 获取雅克比矩阵
	 * @param q [in] 关节数值
	 * @return 当关节为Q时机器人的雅克比矩阵
	 */
	Jacobian getJacobian(const robot::math::Q& q) const;

	/** @brief 获取关节数值 */
	const robot::math::Q getQ() const;

	/** @brief 设置关节数值 */
	void setQ(robot::math::Q);

//	Config getConfig(const Q&) const;

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

	/**
	 * @brief 获取tool
	 * @return tool的地址
	 */
	Frame* getTool() const;

	/**
	 * @brief 返回关节下限
	 * @return 关节下限
	 */
	Q getJointMin() const;

	/**
	 * @brief 返回关节上限
	 * @return 关节上限
	 */
	Q getJointMax() const;

	/**
	 * @brief 判断关节数值是否合理
	 * @param joint [in] 关节数值
	 * @retval true 关节数值在各个关节设定范围内
	 * @retval false 关节数值不在关节设定范围内
	 * @deprecated fixJoint instead.
	 */
	bool isJointValid(const Q& joint) const;

	/**
	 * @brief 关节范围判断与处理
	 * @param joint [in] 要处理的关节, 范围是-180~180
	 * @return 范围内的关节解. 例如范围是-360 - 360, 判断的角度是30, 则将30和-330都添加到结果中
	 *
	 * only for dof=6
	 */
	std::vector<Q> fixJoint(Q& joint) const;

	/** @brief 格式化打印
	 * @todo 未定义功能
	 */
	void print();

	virtual ~SerialLink();
private:
	/** @brief 模型名字 */
	const std::string _name;

	/** @brief 关节地址列表 */
	std::vector<Link::ptr> _linkList;

	/** @brief 基坐标系 */
	Frame _worldFrame;

	/** @brief 末端到工具中心点的偏置 */
	Frame* _endToTool;

	/** @brief 默认工具
	 *
	 * 法兰 */
	Frame _defaultTool;

};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* SERIALLINK_H_ */
