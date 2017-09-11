/**
 * @brief Config类
 * @date Aug 29, 2017
 * @author a1994846931931
 */

#ifndef CONFIG_H_
#define CONFIG_H_

# include "../math/Q.h"

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */

/**
 * @brief 逆运算取优的配置类
 *
 * > 参考史陶比尔Val3语言中的Trsf变量, 定义了肘肩腕的模式. <br>
 * - 对于肩部, 有 **righty** , **lefty** , **ssame** , **sfree** 四种配置. 其中<br>
 * 	-  **righty** 的定义为: <br>
 * 		- @f$ a_2 + a_3*cos(j_2) + a_4*cos(j_2 + j_3) - d_4*sin(j_2 + j_3) < 0 @f$ <br>
 * 	-  **lefty** 的定义为: <br>
 * 		- @f$ a_2 + a_3*cos(j_2) + a_4*cos(j_2 + j_3) - d_4*sin(j_2 + j_3) >= 0 @f$ <br>
 * 	- **ssame** 表示与当前机器人的配置相同
 * 	- **sfree** 表示肩部没有限制.<br>
 *
 * - 对于肘部, 有 **epositive** , **enegative** , **esame** , **efree** 四种配置, 其中 <br>
 * 	- **epositive** 的定义为: <br>
 * 		- @f$ j_3 >= 0 @f$<br>
 * 	- **enegative** 的定义为: <br>
 * 		- @f$ j_3 < 0 @f$ <br>
 * 	- **esame** 表示与当前机器人的配置相同 <br>
 * 	- **efree** 表示肘部没有限制. <br>
 *
 * - 对于腕部, 有 **wpositive** , **wnegative** , **wsame** , **wfree** 四种配置, 其中
 * 	- **wpositive** 的定义为: <br>
 * 		- @f$ j_5 >= 0 @f$<br>
 * 	- **wnegative** 的定义为: <br>
 * 		- @f$ j_5 < 0 @f$ <br>
 * 	- **wsame** 表示与当前机器人的配置相同
 * 	- **wfree** 表示腕部没有限制.<br>
 */
class Config {
public:
	static const int righty=2, lefty=1, ssame=0, sfree=-1;
	static const int epositive=2, enegative=1, esame=0, efree=-1;
	static const int wpositive=2, wnegative=1, wsame=0, wfree=-1;
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 默认配置为{ssame, esame, wsame}
	 */
	Config();

	/**
	 * @brief 构造配置
	 * @param shoulder [in] 肩部配置
	 * @param elbow [in] 肘部配置
	 * @param wrist [in] 腕部配置
	 *
	 * 使用Config中定义的const常亮进行构造. 举例代码如下
	 * @code
	 * robot::model::Config config(Config::ssame, Config::esame, Config::wsame)
	 * @endcode
	 */
	Config(int shoulder, int elbow, int wrist);

	/**
	 * @brief 设置肩部配置
	 * @param shoulder [in] 可用参数:
	 * - Config::righty
	 * - Config::lefty
	 * - Config::ssame
	 * - Config::sfree
	 */
	void setShoulder(int);

	/**
	 * @brief 设置肘部配置
	 * @param elbow [in] 可用参数:
	 * - Config::epositive
	 * - Config::enegative
	 * - Config::esame
	 * - Config::efree
	 */
	void setElbow(int);

	/**
	 * @brief 设置腕部配置
	 * @param wrist [in] 可用参数:
	 * - Config::wpositive
	 * - Config::wnegative
	 * - Config::wsame
	 * - Config::wfree
	 */
	void setWrist(int);

	/**
	 * @brief 返回肩部参数
	 * @return
	 * - Config::righty = 2
	 * - Config::lefty  = 1
	 * - Config::ssame  = 0
	 * - Config::sfree  = -1
	 */
	int getShoulder() const;

	/**
	 * @brief 返回肘部参数
	 * @return
	 * - Config::epositive = 2
	 * - Config::enegative = 1
	 * - Config::esame     = 0
	 * - Config::efree     = -1
	 */
	int getElbow() const;

	/**
	 * @brief 返回腕部参数
	 * @return
	 * - Config::wpositive = 2
	 * - Config::wnegative = 1
	 * - Config::wsame     = 0
	 * - Config::wfree     = -1
	 */
	int getWrist() const;

	/**
	 * @brief 判断与config是否相等
	 * @param config [in] 判断相等的数
	 * @retval true 四个参数都相同
	 * @retval false 有不同的参数
	 */
	bool operator==(const Config& config) const;

	/**
	 * @brief 判断与config是否相等
	 * @param config [in] 判断相等的数
	 * @retval true 有不同的参数
	 * @retval false 四个参数都相同
	 */
	bool operator!=(const Config& config) const;
	virtual ~Config();
private:
	/** @brief 肩部配置 */
	int _shoulder;

	/** @brief 肘部配置 */
	int _elbow;

	/** @brief 腕部配置 */
	int _wrist;
};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* CONFIG_H_ */
