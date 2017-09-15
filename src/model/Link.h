/**
 * @brief Link连杆类DH参数
 * @date Aug 14, 2017
 * @author zrf
 */

#ifndef LINK_H_
#define LINK_H_
# include "../math/HTransform3D.h"
# include "../kinematics/Frame.h"
# include "DHParameters.h"

using namespace robot::kinematic;

namespace robot {
namespace model {

/** @addtogroup model
 * @brief 机器人连杆文件
 * @{
 */

/**
 * @brief 包括连杆定义、连杆转动范围、连杆DH变换阵、连杆的初始化
 *
 */

class Link {
public:
	/**
	 * @brief 连杆构造函数定义，生成初始DH变换阵
	 * @param alpha [in]
	 * @param a [in]
	 * @param d [in]
	 * @param theta [in]
	 * @param min [in]
	 * @param max [in]
	 * @param sigma [in]
	 * @warning sigma为连杆类型，当sigma为0：转动副；sigma为1：移动副
	 * min、max 为转动最小最大角度或伸缩长度
	 */
	Link(double alpha, double a, double d, double theta, double min, double max, bool sigma=0);

	/**
	* @brief 连杆增量变化阵
	* @param offset [in] 增加的角度或者长度
	*/
	void change(double offset);

	/**
	* @brief 返回初始状态
	*/
	void reset();

	/**
	* @brief 返回一个Frame
	*
	*/
	Frame* getFrame();

	/**
	* @brief 获取参数a
	* @return 返回参数a
	*/
	const double a() const;

	/**
	* @brief 获取参数d
	* @return 返回参数d
	*/
	const double d() const;

	/**
	* @brief 获取参数theta
	* @return 返回参数theta
	*/
	const double theta() const;

	/**
	* @brief 获取参数alpha
	* @return 返回参数alpha
	*/
	const double alpha() const;

	/**
	 * @brief 返回关节下限
	 * @return 关节最小值
	 */
	double lmin() const;

	/**
	 * @brief 返回关节上限
	 * @return 关节最大值
	 */
	double lmax() const;

	/**
	 * @brief 返回sin(alpha)
	 * @return  @f$ sin(\alpha) @f$
	 */
	inline double sa() const
	{
		return _sa;
	}

	/**
	 * @brief 返回cos(alpha)
	 * @return  @f$ cos(\alpha) @f$
	 */
	inline double ca() const
	{
		return _ca;
	}

	/**
	* @brief 获取DH参数类
	* @return 返回DH参数类
	*/
	const DHParameters& getDHParams() const;

	/**
	* @brief 依据增量生成变换阵
	* @return 返回DH参数
	*/
	HTransform3D<> getTransform(double q) const;

	/** @brief 返回关节值 */
	inline double getQ(){return _offset;}
	virtual ~Link();

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	bool _sigma;
	double _offset;
	double _lmin;
	double _lmax;
	Frame* _frame;
	DHParameters _dHParam;
	const double _sa;
	const double _ca;;
	const double _st;
	const double _ct;
};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* LINK_H_ */
