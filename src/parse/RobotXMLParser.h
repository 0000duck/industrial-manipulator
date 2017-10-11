/*
 * RobotXMLParser.h
 *
 *  Created on: Oct 10, 2017
 *      Author: a1994846931931
 */

#ifndef ROBOTXMLPARSER_H_
#define ROBOTXMLPARSER_H_

# include "../model/SerialLink.h"

using namespace robot::model;

namespace robot {
namespace parse {

class RobotXMLParser {
public:
	RobotXMLParser();
	SerialLink::ptr parse(const char* filename);
	SerialLink::ptr parse(const std::string filename);
	virtual ~RobotXMLParser();

};

} /* namespace parse */
} /* namespace robot */

#endif /* ROBOTXMLPARSER_H_ */
