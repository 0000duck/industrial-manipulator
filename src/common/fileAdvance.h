/**
 * @brief fileAdvance
 * @date Oct 12, 2017
 * @author a1994846931931
 */

#ifndef FILEADVANCE_H_
#define FILEADVANCE_H_

# include "../math/Q.h"
# include "../math/HTransform3D.h"

namespace robot{
namespace common{

/**
 * @addtogroup common
 * @{
 */
using std::vector;
using namespace robot::math;

/**
 * @brief 保存Q路径
 * @param filename [in] 文件名(包含路径)
 * @param qPath [in] 要保存的路径
 * @return 是否成功保存
 *
 * 每行保存的格式为 "q1,q2,q3,q4,q5,q6;\n"
 */
bool saveQPath(const char* filename, vector<Q>& qPath);

/**
 * @brief 保存含时间的Q路径
 * @param filename [in] 文件名(包含路径)
 * @param qPath [in] 要保存的路径
 * @param vt [in] 时间向量
 * @return 是否保存成功
 *
 * 每行保存的格式为 "q1,q2,q3,q4,q5,q6,t;\n"
 */
bool saveQPath(const char* filename, vector<Q>& qPath, vector<double>& vt);

/**
 * @brief 保存Vector3D<double>路径
 * @param filename [in] 文件名(包含路径)
 * @param vPath [in] 要保存的路径
 * @return 是否成功保存
 *
 * 每行保存的格式为 "v1,v2,v3\n"
 */
bool savePosPath(const char* filename, vector<Vector3D<double>>& vPath); //,,格式

/**
 * @brief 保存double路径
 * @param filename [in] 文件名(包含路径)
 * @param doublePath [in] 要保存的double数据
 * @param time [in] 对应的时间列表
 * @return 是否成功保存
 *
 * 每行保存的格式为 "num,t\n"
 */
bool saveDoublePath(const char* filename, const vector<double>& doublePath, const vector<double>& time);

/** @} */
}
}

#endif /* FILEADVANCE_H_ */
