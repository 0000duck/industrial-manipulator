/*
 * printAdvance.h
 *
 *  Created on: Aug 11, 2017
 *      Author: a1994846931931
 */

#ifndef _PRINTADVANCE_H_
#define _PRINTADVANCE_H_

#include <sstream>

namespace robot{
namespace common{

template < typename T > std::string to_string( const T& n )
{
	std::ostringstream stm ;
	stm << n ;
	return stm.str() ;
}

template <typename T>
void println(const T& printable)
{
	std::cout<<to_string(printable)<<'\n';
}

void println();
}
}

#endif /* _PRINTADVANCE_H_*/
