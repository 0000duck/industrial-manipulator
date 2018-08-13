/**
 * @brief Tool.h
 * @date Nov 15, 2017
 * @author a1994846931931
 */

#ifndef TOOL_H_
#define TOOL_H_

# include "../math/Q.h"
# include "SerialLink.h"

using robot::math::Q;
using std::vector;
using robot::model::SerialLink;

namespace robot {
namespace model {

/** @addtogroup model
 * @brief 机器人模型的表达, DH参数, 雅克比矩阵等.
 */

/**
 * @brief 工具坐标计算类
 */
class Tool {
public:
	Tool();
	virtual ~Tool();
public:
	/**
	 * @brief 获取工具坐标中心点(TCP).
	 *
	 * 将工具坐标的中心点多次触碰统一参考点, 获取每次的关节角度值, 就可以通过最小二乘法
	 * 计算TCP的位置
	 *
	 * @param joints [in] 几次触碰的关节角
	 * @param robot [in] 机器人模型
	 * @param TCP [out] 获取的TCP
	 * @retval true 成功计算
	 * @retval false 未能成功计算. 可能是因为点数不够, 可以换一组点再尝试.
	 */
	static bool toolGetTCP(vector<Q> joints, SerialLink::ptr robot, Vector3D<>& TCP);
};

/**@} */

} /* namespace model */
} /* namespace robot */

#endif /* TOOL_H_ */
