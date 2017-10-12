/*
 * fileAdvance.h
 *
 *  Created on: Oct 12, 2017
 *      Author: a1994846931931
 */

#ifndef FILEADVANCE_H_
#define FILEADVANCE_H_

# include "../math/Q.h"

namespace robot{
namespace common{

using std::vector;
using robot::math::Q;

bool saveQPath(const char* filename, vector<Q>& qPath);

bool saveDoublePath(const char* filename, vector<double>& doublePath, vector<double>& time);

}
}

#endif /* FILEADVANCE_H_ */
