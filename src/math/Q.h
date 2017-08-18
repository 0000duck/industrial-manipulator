/*
 * Q.h
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

#ifndef Q_H_
#define Q_H_

# include <stddef.h>
# include <vector>

namespace robot {
namespace math {

class Q {
public:
	Q();
	Q(double, double, double, double, double, double);
//	Q(Q&);
	int size();
	double& operator()(int);
	double& operator[](int);
	virtual ~Q();
public:
	static Q& zero(int);
private:
	int _size;
	std::vector<double> _value;
};

} /* namespace math */
} /* namespace robot */

#endif /* Q_H_ */
