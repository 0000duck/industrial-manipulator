/*
 * DHParameters.cpp
 *
 *  Created on: Aug 17, 2017
 *      Author: a1994846931931
 */

# include "DHParameters.h"
# include <iostream>

namespace robot {
namespace model {

DHParameters::DHParameters(double alpha, double a, double d, double theta) :
	_alpha(alpha),_a(a),_d(d),_theta(theta){
	// TODO Auto-generated constructor stub

}

double DHParameters::alpha() const
{
	return _alpha;
}

double DHParameters::a() const
{
	return _a;
}

double DHParameters::d() const
{
	return _d;
}

double DHParameters::theta() const
{
	return _theta;
}

void DHParameters::print() const
{
	std::cout << this->alpha() << "\t" << this->a() << "\t" << this->d() << "\t" << this->theta() << std::endl;
}

DHParameters::~DHParameters() {
	// TODO Auto-generated destructor stub
}

} /* namespace model */
} /* namespace robot */
