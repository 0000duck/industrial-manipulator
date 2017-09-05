/**
 * @brief 单位四元数类Quaternion
 * @date Aug 24, 2017
 * @author a1994846931931
 */

#ifndef QUATERNION_H_
#define QUATERNION_H_

//# include "Vector3D.h"
# include "Rotation3D.h"
# include "HTransform3D.h"

namespace robot {
namespace math {

/** @addtogroup math
 * @{
 */
class Quaternion {
	typedef struct{
		double theta;
		Vector3D<double> n;
	} rotVar;
public:
	Quaternion();
	Quaternion(double, double, double, double);
	Quaternion(const Quaternion&);
	Quaternion(const Rotation3D<double>&);
	Quaternion(const HTransform3D<double>&);
	Quaternion(double theta, const Vector3D<double>& n);
	Quaternion operator+(const Quaternion&) const;
	Quaternion operator-(const Quaternion&) const;
	Quaternion operator*(const Quaternion&) const;
	void operator+=(const Quaternion&);
	void operator-=(const Quaternion&);
	void operator*=(const Quaternion&);
	void operator=(const Quaternion&);
	bool operator==(const Quaternion&) const;
	bool operator!=(const Quaternion&) const;
	double r() const;
	double i() const;
	double j() const;
	double k() const;
	Quaternion conjugate() const;
	double norm() const;
	void normalize();
	robot::math::Rotation3D<double> toRotation3D() const;
	robot::math::HTransform3D<double> toHTransform3D() const;
	rotVar getRotationVariables() const;
	void print() const;
public:
	static Quaternion DH(double alpha, double theta);
	virtual ~Quaternion();
private:
	double _r, _i, _j, _k;
};

/** @} */
} /* namespace math */
} /* namespace robot */

#endif /* QUATERNION_H_ */
