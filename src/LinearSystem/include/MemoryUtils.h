#ifndef __LINEAR__SYSTEM__LIB__MEMORY__UTILS__H__
#define __LINEAR__SYSTEM__LIB__MEMORY__UTILS__H__

namespace LinearSystemLib
{
	template < class Type >
		inline Type** Allocate2DArray( int dim1, int dim2 )
	{
		Type** array = new Type* [dim1];

		const size_t size = sizeof(Type) * dim2;

		for( int i = 0 ; i < dim1 ; ++i )
		{
			array[i] = new Type[dim2];
			memset( array[i], 0, size );
		}

		return array;
	}

	template < class Type >
		inline void Delete2DArray( Type** &mem, int dim1 )
	{
		if( mem )
		{
			for( int i = 0 ; i < dim1 ; ++i )
				delete[] mem[i];
			delete[] mem;
			mem = NULL;
		}
	}
}

#endif