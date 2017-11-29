/**
 * @brief RobotXMLParser.h
 * @date Oct 10, 2017
 * @author a1994846931931
 */

#ifndef ROBOTXMLPARSER_H_
#define ROBOTXMLPARSER_H_

# include "../model/SerialLink.h"

using namespace robot::model;

namespace robot {
namespace parse {

/**
 * @addtogroup parse
 * @brief 文件解析器
 * @{
 */

/**
 * @brief 机器人模型文件解析器
 */
class RobotXMLParser {
public:
	RobotXMLParser();
	virtual ~RobotXMLParser();
public:
	/**
	 * @brief 解析机器人模型文件
	 * @param filename [in] 文件名(包含路径)
	 * @return 机器人模型指针
	 */
	static SerialLink::ptr parse(const char* filename);

	/**
	 * @brief 解析机器人模型文件
	 * @param filename [in] 文件名(包含路径)
	 * @return 机器人模型指针
	 */
	static SerialLink::ptr parse(const std::string filename);
};

/**@}*/

} /* namespace parse */
} /* namespace robot */

#endif /* ROBOTXMLPARSER_H_ */
