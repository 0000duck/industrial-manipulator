/*
 * DHTable.h
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#ifndef DHTABLE_H_
#define DHTABLE_H_

# include "DHParameters.h"
# include <vector>

namespace robot {
namespace model {

class DHTable {
public:
	DHTable();
	int size();
	const DHParameters& operator()(int) const;
	const DHParameters& operator[](int) const;
	void append(const DHParameters&);
	virtual ~DHTable();
private:
	std::vector<DHParameters> _dHParam;
private:

};

} /* namespace model */
} /* namespace robot */

#endif /* DHTABLE_H_ */
