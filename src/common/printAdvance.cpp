/*
 * printAdvanve.cpp
 *
 *  Created on: Aug 15, 2017
 *      Author: a1994846931931
 */

# include <sstream>
# include <iostream>

namespace robot{
namespace common{



template <typename T>
void println(const T& printable)
{
	std::cout<<printable<<'\n';
}
void println()
{
	std::cout<<'\n';
}

}
}
