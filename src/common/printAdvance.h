/**
 * @brief 常用的print函数
 * @date Aug 11, 2017
 * @author a1994846931931
 */

#ifndef _PRINTADVANCE_H_
#define _PRINTADVANCE_H_

# include <sstream>
# include <iostream>
# include <iomanip>

using std::cout;
using std::setfill;
using std::setw;
using std::endl;

namespace robot{
namespace common{

/** @addtogroup common
 * @{ */

/**
 * @brief 转换成std::string类型.
 * @param n [in] 任意支持类型的输入
 * @return n 转换成的字符串
 */
template < typename T > std::string to_string( const T& n )
{
	std::ostringstream stm ;
	stm << n ;
	return stm.str() ;
}

/**
 * @brief 打印一行
 * @param printable [in] 任意可以由robot::common::to_string函数转换成字符串的变量
 */
template <typename T>
void println(const T& printable)
{
	cout<<to_string(printable)<<'\n';
}

/**
 * @brief 打印空行
 */
void println();
}
}

/** @} */
#endif /* _PRINTADVANCE_H_*/
