#include "SparseLinearSystem.h"

namespace LinearSystemLib
{
	SparseLinearSystem::SparseLinearSystem( StableSparseMatrix* pA , double** pB, unsigned int _dim )
		: LinearSystem()
	{
		mp_A = pA;
		mp_B = pB;
		mui_Dimension = _dim;
	}

	SparseLinearSystem::~SparseLinearSystem()
	{
		delete mp_A;
		mp_A = 0;

		if( mp_B )
		{
			for( unsigned int i = 0u ; i < mui_Dimension ; ++i )
				delete[] mp_B[i];
			delete[] mp_B;		
		}

		mp_B = 0;
		mui_Dimension = 0;
	}
}