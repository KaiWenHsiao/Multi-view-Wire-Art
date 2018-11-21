#ifndef _SPARSE_LINEAR_SYSTEM_H_
#define _SPARSE_LINEAR_SYSTEM_H_

#include "LinearSystem.h"
#include "StableSparseMatrix.h"

namespace LinearSystemLib
{
	class SparseLinearSystem : public LinearSystem
	{
	private:
		SparseLinearSystem( const SparseLinearSystem& );
		SparseLinearSystem& operator= ( const SparseLinearSystem& );

	public:

		/// constructor.
		/// 在此的 pA, pB 要傳 new 出來的 object, memory 將由此 object 負責釋放,
		SparseLinearSystem( StableSparseMatrix* pA  = 0, double** pB = 0, unsigned int _dim = 0 );
		virtual ~SparseLinearSystem();

		/// Get Matrix A.
		StableSparseMatrix*& GetA()					{ return mp_A; }
		const StableSparseMatrix* GetA() const		{ return mp_A; }
		
		/// Get Matrix B.
		double**& GetB()				{ return mp_B; }
		const double** GetB() const 	{ return const_cast<const double**>(mp_B); }

		unsigned int Dimension() const	{ return mui_Dimension; }
		unsigned int& Dimension()		{ return mui_Dimension; }

	protected:
		StableSparseMatrix*	mp_A;

		// storing the matrix B. 
		// it can be like this : [B] or [Bx By] or [Bx By Bz] etc..
		// their dimension is 1, 2, 3, etc..
		double**	mp_B;

		unsigned int mui_Dimension;
	};
}

#endif