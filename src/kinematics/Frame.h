/**
 * @brief Frame类
 * @date Aug 11, 2017
 * @author a1994846931931
 */

#ifndef FRAME_H_
#define FRAME_H_

# include <vector>
# include "stddef.h"
# include "../math/HTransform3D.h"

using namespace robot::math;

namespace robot {
namespace kinematic {

/** @addtogroup kinematics
 * @brief 机器人运动学模型.
 *
 * 包括的类有:
 * 1. Frame: 用以描述一个坐标系
 * 2. State: 描述机器人的状态(关节角度, 速度, 加速度等)
 * 3. Trsf: 实现XYZ固定角坐标系和xyz位置描述位姿的方法
 * @{
 */

/**
 * @brief Frame类, 定义一个坐标系
 *
 * 每个Frame可以定义一个父Frame和数个子Frame. <br>
 * 每个Frame都对应一个变换矩阵. <br>
 * 每个Frame都有一个唯一的ID号来标识它.
 */
class Frame {
	/**@brief 定义Frame指针的vector容器类
	 */
	typedef std::vector<Frame*> FrameList;
public:
	/**
	 * @brief 默认构造函数.
	 *
	 * 父为空(NULL), 变换矩阵由其默认构造函数构造.
	 */
	Frame();

	/**
	 * @brief 指定变换矩阵的构造函数.
	 * @param transform [in] 指定的变换矩阵
	 *
	 * 父为空(NULL).
	 */
	Frame(HTransform3D<double>& transform);

	/**
	 * @brief 指定父Frame和变换矩阵的构造函数.
	 * @param parent [in] 父Frame的地址
	 * @param  transform [in] 指定的变换矩阵
	 */
	Frame(Frame* parent, HTransform3D<double>& transform);

	/**
	 * @brief 指定父Frame的构造函数.
	 * @param parent [in] 父Frame的地址
	 *
	 * 变换矩阵由其默认构造函数构造.
	 */
	Frame(Frame* parent);

	/**
	 * @brief 为Frame添加一个子Frame.
	 * @param child [in] 子Frame的地址
	 * @param doSetParent 是否要设置子Frame的parent为this
	 *
	 * 一般外部使用函数时, doSetParent默认设置为true, 这样在给Frame添加child的同时也会设置该child的父为this.
	 * 这样可以保证数据的一致性. 因此外部调用函数的时候应默认其为true. 设置这个参数的意义在于, 内部避免循环调用.
	 */
	void addChild(Frame* child, bool doSetParent=true);

	/**
	 * @brief 为Frame设置它的父Frame.
	 * @param parent [in] 父Frame的地址
	 * @param doCheckParent 是否要把这个Frame添加到它的子Frame列表中
	 *
	 * 一般外部使用函数时, doCheckParent默认设置为true, 这样在给Frame设置parent的同时也会添加自己到该parent的子列表中去.
	 * 这样可以保证数据的一致性. 因此外部调用函数的时候应默认其为true. 设置这个参数的意义在于, 内部避免循环调用.
	 */
	void setParent(Frame* parent, bool doCheckParent=true);

	/**
	 * @brief 移除父Frame信息.
	 * @param doRemoveChild [in] 是否要把这个Frame从它原来parent的子列表中移除
	 *
	 * 一般外部使用函数时, doRemoveChild默认设置为true, 这样在给Frame移除parent的同时也会将自身从parent的子列表中移除
	 * 这样可以保证数据的一致性. 因此外部调用函数的时候应默认其为true. 设置这个参数的意义在于, 内部避免循环调用.
	 */
	void removeParent(bool doRemoveChild=true);

	/**
	 * @brief 移除一个子Frame.
	 * @param child [in] 要移除的子Frame的地址
	 * @param doRemoveParent [in] 是否要清除掉这个子Frame的父Frame信息
	 *
	 * 一般外部使用函数时, doRemoveParent默认设置为true, 这样在操作同时也会清除这个子Frame的父Frame信息
	 * 这样可以保证数据的一致性. 因此外部调用函数的时候应默认其为true. 设置这个参数的意义在于, 内部避免循环调用.
	 */
	void removeChild(Frame* child, bool doRemoveParent=true);

	/**
	 * @brief 获取父Frame
	 * @return 返回的是父Frame的地址, 若没有, 返回的是NULL
	 */
	const Frame* getParent();

	/**
	 * @brief 获取子列表
	 * @return 返回的是vector<Frame*>类型的列表
	 */
	const FrameList& getChildren();

	/**
	 * @brief 判断子列表中是否包含一个Frame
	 * @param child [in] 待判断的Frame
	 * @retval true 子列表包含child
	 * @retval true 子列表不包含child
	 */
	bool haveChild(Frame* child);

	/**
	 * @brief 获取Frame在子列表中的索引位置
	 * @param child [in] 要判断的Frame地址
	 * @retval -1 找不到该Frame
	 * @retval 非负整数 该Frame在子列表中的索引位置
	 */
	int getChildIndex(Frame* child);

	/**
	 * @brief 设置变换矩阵
	 * @param transform [in] 要设置的变换矩阵
	 */
	void setTransform(HTransform3D<double>& transform);

	/**
	 * @brief 显式地更新变换矩阵的值
	 * @param a11 [in]
	 * @param a12 [in]
	 * @param a13 [in]
	 * @param a14 [in]
	 * @param a21 [in]
	 * @param a22 [in]
	 * @param a23 [in]
	 * @param a24 [in]
	 * @param a31 [in]
	 * @param a32 [in]
	 * @param a33 [in]
	 * @param a34 [in]
	 *
	 * <br>更新后变换矩阵的值为:<br>
	 * @f$
	 * \left[
	 * \begin{array}{cccc}
	 * a11 & a12 & a13 & a14 \\
	 * a21 & a22 & a23 & a24 \\
	 * a31 & a32 & a33 & a34 \\
	 * 0 & 0 & 0 & 1
	 * \end{array}
	 * \right]
	 * @f$
	 */
	void updateTransform(const double& a11, const double& a12, const double& a13, const double& a14,
			const double& a21, const double& a22, const double& a23, const double& a24,
			const double& a31, const double& a32, const double& a33, const double& a34);

	/**
	 * @brief 获取变换矩阵
	 * @return 返回变换矩阵
	 */
	const HTransform3D<double>& getTransform() const;

	/**
	 * @brief 打印
	 *
	 * 按照如下格式打印: <br>
	 * Frame ID: <ID> <br>
	 * Have parent? -> <yes, no> <br>
	 * Number of children -> <size> <br>
	 * transform: <br>
	 * <transform>
	 */
	void print();
	virtual ~Frame();
private:
	/**
	 * @brief Frame类对象计数器
	 */
	static int _frameIDCounter;

	/**
	 * @brief 具体对象的ID号
	 */
	int _frameID;

	/**
	 * @brief Frame的变换矩阵
	 */
	HTransform3D<double> _tran;

	/**
	 * @brief 父Frame地址
	 */
	Frame* _parent;

	/**
	 * @brief 子Frame的地址列表
	 */
	FrameList _children;
};

/** @} */
} /* namespace kinematic */
} /* namespace robot */

#endif /* FRAME_H_ */
