/*
 * Jacobian.h
 *
 *  Created on: Aug 24, 2017
 *      Author: a1994846931931
 */

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

# include "../ext/Eigen/Dense"

namespace robot {
namespace model {

class Jacobian {
public:
	Jacobian();
	Jacobian(double (&j)[6][6]);
	void doInverse();
	Jacobian inverse() const;
	int rank() const;
	void print() const;
	virtual ~Jacobian();
private:
	Jacobian(Eigen::MatrixXd matrix): _j(6, 6)
	{
		_j = matrix;
	}
	Eigen::MatrixXd getMatrix() const
	{
		return _j;
	}
private:
	Eigen::MatrixXd _j;
};

} /* namespace model */
} /* namespace robot */

#endif /* JACOBIAN_H_ */
