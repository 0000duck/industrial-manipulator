/*
 * test.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

# include "math/Rotation3D.h"
# include "math/HTransform3D.h"
# include "kinematics/Trsf.h"
# include <iostream>
# include <math.h>
# include <string>

using namespace robot::math;
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
void println(const T& printable){
	std::cout<<printable<<'\n';
}
int main(){
	robot::kinematic::Trsf trsf1(0,1,0,M_PI,0,0);
	trsf1.getTransform().print();
	return 0;
}



