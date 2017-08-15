/*
 * test.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

# include "math/Rotation3D.h"
# include "math/HTransform3D.h"
# include "kinematics/Trsf.h"
# include "kinematics/Frame.h"
# include <iostream>
# include <math.h>
# include <string>

using namespace robot::math;
using namespace robot::kinematic;
using std::cout;
#include <sstream>

namespace patch
{
    template < typename T > std::string to_string( const T& n )
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str() ;
    }
}

template <typename T>
void println(const T& printable)
{
	std::cout<<printable<<'\n';
}
void println()
{
	std::cout<<'\n';
}
int main(){
	robot::kinematic::Trsf trsf1(0,1,0,M_PI,0,0);
	HTransform3D<double>& transform1 = trsf1.getTransform();
	transform1.print();
	// update matrix using HTransform3D::setRotation
	println();
	transform1.setRotation(1,2,3,4,5,6,7,8,9);
	transform1.print();
	Frame frame1(&transform1);
	// update matrix using Frame::updateTransform
	println();
	frame1.updateTransform(1,1,1,1,2,2,2,2,3,3,3,3);
	frame1.getTransform()->print();

	return 0;
}



