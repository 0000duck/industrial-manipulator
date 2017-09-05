/**
 * @brief Link类
 * @date Aug 14, 2017
 * @author zrf
 */

#ifndef LINK_H_
#define LINK_H_
# include "../math/HTransform3D.h"
# include "../kinematics/Frame.h"
# include "DHParameters.h"

using namespace robot::kinematic;

namespace robot {
namespace model {

/** @addtogroup model
 * @{
 */
class Link {
public:
//	Link();
	Link(double alpha, double a, double d, double theta, double min, double max, bool sigma=0);
	void change(double offset);
	void reset();
	Frame* getFrame();
	const double a() const;
	const double d() const;
	const double theta() const;
	const double alpha() const;
	const DHParameters& getDHParams() const;
	HTransform3D<> getTransform(double q) const;
	inline double getQ(){return _offset;}
	virtual ~Link();

private:
	double _theta;
	double _d;
	double _a;
	double _alpha;
	bool _sigma;
	double _offset;
	double _lmin;
	double _lmax;
	Frame* _frame;
	DHParameters _dHParam;

};

/** @} */
} /* namespace model */
} /* namespace robot */

#endif /* LINK_H_ */
