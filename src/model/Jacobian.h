/**
 * @brief Jacobian类
 * @date Aug 24, 2017
 * @author a1994846931931
 */

#ifndef JACOBIAN_H_
#define JACOBIAN_H_

# include "../ext/Eigen/Dense"
# include "../math/Q.h"
# include <vector>

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */
class Jacobian {
public:
	Jacobian();
	Jacobian(std::vector< std::vector<double> >);
	void doInverse();
	Jacobian inverse() const;
	int rank() const;
	void print() const;
	void update(const double (&j)[6][6]);
	double operator()(int, int) const;
	void operator=(const Jacobian&);
	robot::math::Q operator*(const robot::math::Q& jointVelocity) const;
	int size() const;
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
	int _size;
	Eigen::MatrixXd _j;
};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* JACOBIAN_H_ */
