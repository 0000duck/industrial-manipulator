/*
 * DHTable.h
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#ifndef DHTABLE_H_
#define DHTABLE_H_

# include "DHParameters.h"

namespace robot {
namespace model {

class DHTable {
public:
	DHTable();
	const DHParameters& operator()(int) const;
	virtual ~DHTable();
private:

};

} /* namespace model */
} /* namespace robot */

#endif /* DHTABLE_H_ */
