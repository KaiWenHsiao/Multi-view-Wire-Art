#ifndef _LINEAR_SYSTEM_H_
#define _LINEAR_SYSTEM_H_

#include "Prerequistites.h"

namespace LinearSystemLib
{
	_interface LinearSystem
	{
	protected:
		LinearSystem() {;}
		LinearSystem( const LinearSystem& ) {;}

	public:
		virtual ~LinearSystem() {;}
	};
}


#endif