/**
 * @brief DHTable类
 * @date Aug 17, 2017
 * @author a1994846931931
 */

#ifndef DHTABLE_H_
#define DHTABLE_H_

# include "DHParameters.h"
# include <vector>

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */

/**
 * @brief DH表
 */
class DHTable {
public:
	/**
	 * @brief 默认构造函数
	 *
	 * 构造一个空的DH表
	 */
	DHTable();

	/** @brief 获取DH表的行数 */
	int size();

	/** @brief 获取某一行的DH参数 */
	const DHParameters& operator()(int) const;

	/** @brief 获取某一行的DH参数 */
	const DHParameters& operator[](int) const;

	/** @brief 添加一行DH参数 */
	void append(const DHParameters&);

	/** @brief 格式化打印 */
	void print() const;
	virtual ~DHTable();
private:
	/** @brief DH参数表 */
	std::vector<DHParameters> _dHParam;
private:

};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* DHTABLE_H_ */
