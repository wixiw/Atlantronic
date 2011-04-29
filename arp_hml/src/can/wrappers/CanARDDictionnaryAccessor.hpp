/*
 * CanARDDictionnaryAccessor.hpp
 *
 *  Created on: 29 avr. 2011
 *      Author: wla
 */

#ifndef CANARDDICTIONNARYACCESSOR_HPP_
#define CANARDDICTIONNARYACCESSOR_HPP_

#include <data.h>
#include <string>

using namespace std;

namespace arp_hml
{
	class CanARDDictionnaryAccessor
	{
	public:
		CanARDDictionnaryAccessor();

		static UNS8* getUNS8Pointer(string componentName, string varName);
		static UNS16* getUNS16Pointer(string componentName, string varName);
		static UNS32* getUNS32Pointer(string componentName, string varName);
		static INTEGER16* getINTEGER16Pointer(string componentName, string varName);
		static INTEGER32* getINTEGER32Pointer(string componentName, string varName);
	};
}

#endif /* CANARDDICTIONNARYACCESSOR_HPP_ */
