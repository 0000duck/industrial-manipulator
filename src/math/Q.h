/**h
 * @brief 数组Q类
 * @date Aug 17, 2017
 * @author a1994846931931
 */

#ifndef Q_H_
#define Q_H_

# include <stddef.h>
# include <vector>

namespace robot {
namespace math {

/** @addtogroup math
 * @{
 */

/**
 * @brief Q数组类
 *
 * 本质上是double类型的数组, 为操作方便而建立. 用来描述机器人的关节角度, 速度等信息.
 */
class Q {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 初始化为空的数组
	 */
	Q();

	/**
	 * @brief 构造长度为6的数组
	 *
	 */
	Q(double, double, double, double, double, double);

	/**
	 * @brief 获取数组的长度
	 */
	int size() const;

	/**
	 * @brief 获取数值
	 * @param index [in] 索引位置
	 * @return 获取索引位置变量的非const引用
	 */
	double& operator()(int index);

	/**
	 * @brief 获取数值
	 * @param index [in] 索引位置
	 * @return 获取索引位置变量的值, 要修改数组中的值时, 请用"()"操作
	 */
	double operator[](int index) const;

	/**
	 * @brief 赋值
	 * @param q [in] 赋值数组
	 */
	void operator=(const Q& q);

	/**
	 * @brief 数组相加
	 * @param q [in] 被加数组
	 * @return 对应位置相加, 返回得到的数组
	 */
	Q operator+(const Q& q) const;

	/**
	 * @brief 数组相减
	 * @param q [in] 被减数组
	 * @return 对应位置相减, 返回得到的数组
	 */
	Q operator-(const Q& q) const;

	/**
	 * @brief 数组相乘
	 * @param q [in] 被乘数组
	 * @return 对应位置相乘, 返回得到的数组
	 */
	Q operator*(const Q& q) const;

	/**
	 * @brief 数组相除
	 * @param q [in] 被除数组
	 * @return 对应位置相除, 返回得到的数组
	 */
	Q operator/(const Q& q) const;

	/**
	 * @brief 与常量相加
	 * @param num [in] 加数
	 * @return 分别加num, 返回得到的数组
	 */
	Q operator+(double num) const;

	/**
	 * @brief 与常量相乘
	 * @param num [in] 减数
	 * @return 分别减num, 返回得到的数组
	 */
	Q operator-(double num) const;

	/**
	 * @brief 与常量相乘
	 * @param num [in] 乘数
	 * @return 分别乘以乘数, 返回得到的数组
	 */
	Q operator*(double num) const;

	/**
	 * @brief 与常量相除
	 * @param num [in] 除数
	 * @return 分别乘以除数, 返回得到的数组
	 */
	Q operator/(double num) const;

	/**
	 * @brief 数组相加
	 * @param q [in] 被加数组
	 * @return 对应位置相加, 得到新数组
	 */
	void operator+=(const Q& q);

	/**
	 * @brief 数组相减
	 * @param q [in] 被减数组
	 * @return 对应位置相减, 得到新数组
	 */
	void operator-=(const Q& q);

	/**
	 * @brief 数组相乘
	 * @param q [in] 被乘数组
	 * @return 对应位置相乘, 得到新数组
	 */
	void operator*=(const Q& q);

	/**
	 * @brief 数组相除
	 * @param q [in] 被除数组
	 * @return 对应位置相除, 得到新数组
	 */
	void operator/=(const Q& q);

	/**
	 * @brief 与常量相加
	 * @param num [in] 加数
	 * @return 分别加num, 得到新数组
	 */
	void operator+=(double num);

	/**
	 * @brief 与常量相乘
	 * @param num [in] 减数
	 * @return 分别减num, 得到新数组
	 */
	void operator-=(double num);

	/**
	 * @brief 与常量相乘
	 * @param num [in] 乘数
	 * @return 分别乘以乘数, 得到新数组
	 */
	void operator*=(double num);

	/**
	 * @brief 与常量相除
	 * @param num [in] 除数
	 * @return 分别乘以除数, 得到新数组
	 */
	void operator/=(double num);

	/**
	 * @brief 判断两个数组是否相同
	 * @param q [in] 判断相等的数
	 * @retval true 两个数组相等
	 * @retval false 两个数组不相等
	 */
	bool operator==(const Q& q) const;

	/**
	 * @brief 判断两个数组是否相同
	 * @param q [in] 判断相等的数
	 * @retval true 两个数组不相等
	 * @retval false 两个数组相等
	 */
	bool operator!=(const Q& q) const;

	/**
	 * @brief 比较数组大小
	 * @param q [in] 比较的数组
	 */
	bool operator<(const Q& q) const;

	/**
	 * @brief 比较数组大小
	 * @param q [in] 比较的数组
	 */
	bool operator<=(const Q& q) const;

	/**
	 * @brief 比较数组大小
	 * @param q [in] 比较的数组
	 */
	bool operator>(const Q& q) const;

	/**
	 * @brief 比较数组大小
	 * @param q [in] 比较的数组
	 */
	bool operator>=(const Q& q) const;

	/**
	 * @brief 追加数据
	 * @param num [in] 添加的数据
	 *
	 * 在数组末尾增加一个数据, 数组长度增加1.
	 */
	void pushBack(double num);

	/**
	 * @brief 将数组中的所有元素变成它的绝对值
	 */
	void abs();

	/**
	 * @brief 对于每个位置上的数据, 和min与max对应位置上的数据进行比较, 更新min
	 * 与max中的数据
	 * @param min [out] 比较的最小值数组
	 * @param max [out] 比较的最大值数组
	 * @warning 不对数组长度进行检查, 默认this数组, min和max的长度都是一样的,
	 * 使用这个函数必须要保证这些数组的长度都是相同的. 同时, 如果元素的值小于其在min
	 * 中对应位置上的值, 则默认它不会比max中的要大. 因此使用的时候应当要保证min比max
	 * 对应位置上的数据要小.
	 */
	void doMinmax(Q&min, Q&max);

	/**
	 * @brief 把min中各个位置上的值都更新为min和this中较小的那个
	 * @param min [out] 比较的最小值数组
	 * @warning 不对数组长度进行检查, 默认this数组, 和min的长度都是一样的,
	 * 使用这个函数必须要保证这些数组的长度都是相同的.
	 */
	void doMin(Q&min);

	/**
	 * @brief 把max中各个位置上的值都更新为max和this中较小的那个
	 * @param max [out] 比较的最大值数组
	 * @warning 不对数组长度进行检查, 默认this数组, 和max的长度都是一样的,
	 * 使用这个函数必须要保证这些数组的长度都是相同的.
	 */
	void doMax(Q&max);

	/**
	 * @brief 获取最小的数值
	 * @return 最小的数值
	 */
	double getMin() const;

	/**
	 * @brief 获取最大的数值
	 * @return 最大的数值
	 */
	double getMax() const;

	/**
	 * @brief 判断是否为0数组
	 * @return 是否为0数组
	 */
	bool isZero(double precision=1e-12) const;

	/**
	 * @brief 格式化打印
	 */
	void print() const;
	virtual ~Q();
public:
	/**
	 * @brief 构造纯0数组
	 * @param size [in] 构造数组的长度
	 * @return 返回一个长度为size, 数字全为0的数组.
	 */
	static Q zero(int size);
private:
	/**
	 * @brief 数组大小
	 */
	int _size;

	/**
	 * @brief 数据
	 */
	std::vector<double> _value;
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* Q_H_ */
