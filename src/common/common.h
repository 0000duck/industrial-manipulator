/**
 * @brief 一些常用的函数
 * @date Aug 25, 2017
 * @author a1994846931931
 */

# include <stdlib.h>

#ifndef COMMON_H_
#define COMMON_H_

namespace robot{
namespace common{

/**
 * @addtogroup common
 * @brief 包含常用的函数, 如取随机数, 打印文字等.
 * @{
 */

/**
 * @brief 获得fMin~fMax之间随机数
 * @param fMin [in]
 * @param fMax [in]
 * @return double类型的随机数
 */
double fRand(double fMin, double fMax);
}
}

/** @} */
#endif /* COMMON_H_ */
