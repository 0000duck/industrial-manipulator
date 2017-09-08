/*
 * CompositeInterpolator.h
 *
 *  Created on: Sep 8, 2017
 *      Author: a1994846931931
 */

#ifndef COMPOSITEINTERPOLATOR_H_
#define COMPOSITEINTERPOLATOR_H_

# include "Interpolator.h"
# include "../common/printAdvance.h"
# include <math.h>

namespace robot {
namespace trajectory {

template <class T>
class CompositeInterpolator: public Interpolator<T> {
public:
	CompositeInterpolator(Interpolator<T>* interpolator, Interpolator<double>* mapper)
	{
		_interpolator = Interpolator;
		_mapper = mapper;
		if (fabs(mapper->x(mapper->duration()) - interpolator->duration()) > 1e-3)
			println("警告: 复合插补器, mapper的范围与源插补器的时间周期不一致(误差超出0.001)");
	}

	T x(double t) const
	{
		return _interpolator->x(_mapper->x(t));
	}

	T dx(double t) const
	{
		return _interpolator->dx(_mapper->x(t));
	}

	T ddx(double t) const
	{
		return _interpolator->ddx(_mapper->x(t));
	}

	double duration() const
	{
		return _mapper->duration();
	}
	virtual ~CompositeInterpolator(){}
private:
	Interpolator<T>* _interpolator;
	Interpolator<double>* _mapper;
};

} /* namespace trajectory */
} /* namespace robot */

#endif /* COMPOSITEINTERPOLATOR_H_ */
