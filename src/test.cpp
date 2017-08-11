/*
 * test.cpp
 *
 *  Created on: Aug 10, 2017
 *      Author: a1994846931931
 */

# include "math/Rotation3D.h"
# include <iostream>
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
	Rotation3D<double> a = Rotation3D<double>::identity();
	println("a(1, 0) is " + patch::to_string(a(1, 0))); //（）重载测试
	println("a is:");
	a.print();
	println(&a);
	Rotation3D<double> b = Rotation3D<double>::identity();
	println("b is:");
	b.print();
	println(&b);
	println(a==b? "a equals to b":"a equals to b"); //==重载测试
	println(a.equal(b)? "a equals to b":"a equals to b"); //equal()测试
	Rotation3D<double> c;
	c.multiply(a, b);
	println("multiply(a, b) is:"); //multiply重载测试
	c.print();
	return 0;
}



