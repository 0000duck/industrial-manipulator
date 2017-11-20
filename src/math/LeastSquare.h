/*
 * LeastSquare.h
 *
 *  Created on: Nov 15, 2017
 *      Author: a1994846931931
 */

#ifndef LEASTSQUARE_H_
#define LEASTSQUARE_H_

# include <vector>

using std::vector;

namespace robot {
namespace math {

class LeastSquare {
public:
	LeastSquare();
	virtual ~LeastSquare();
public:
	static bool fromMatrix(const vector<vector<double> > &A, const vector<double> &b, vector<double> &X);
};

} /* namespace model */
} /* namespace robot */

#endif /* LEASTSQUARE_H_ */
