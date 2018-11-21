#include "SparseLinearSystemManager.h"

#include "linearsystemsolver.h"
#include "StableSparseMatrix.h"
#include "MemoryUtils.h"

#include <cassert>

namespace LinearSystemLib
{
	using namespace TaucsLib;

	/// pLinearSystem 將由此物件負責刪除.
	/// pLinearSystem 不可以為 NULL.
	void SparseLinearSystemManager::SetLinearSystem( SparseLinearSystem* pLinearSystem )
	{
		assert( pLinearSystem != 0 );

		bool sol_need_allocate_flag = true;

		if( mp_LinearSystem )
		{
			// 若維度都一樣不要 release solution.
			sol_need_allocate_flag = ( 
				mp_LinearSystem->GetA()->GetNumCols() == pLinearSystem->GetA()->GetNumCols() &&
				mp_LinearSystem->Dimension() == pLinearSystem->Dimension() ) ? false : true;

			// release solution if necessary
			if( sol_need_allocate_flag )
				free_solution( mp_LinearSystem->Dimension() );

			delete mp_LinearSystem;
		}	

		mp_LinearSystem = pLinearSystem;

		// reallocate solution if necessary.
		if( sol_need_allocate_flag )
			allocate_solution( mp_LinearSystem->Dimension(), mp_LinearSystem->GetA()->GetNumCols() );

		freeLLTFactorization();
	}


	void SparseLinearSystemManager::freeMemory()
	{
		if( mp_LinearSystem )
		{
			if( mp_Solution )
				free_solution( Dimension() );

			delete mp_LinearSystem;
			mp_LinearSystem = 0;
		}

		freeLLTFactorization();
	}

	void SparseLinearSystemManager::allocate_solution( unsigned int dim1, unsigned int dim2 )
	{
		mp_Solution = Allocate2DArray<double>( dim1, dim2 );
	}

	void SparseLinearSystemManager::free_solution( unsigned int dim )
	{
		if( mp_Solution )
		{
			for( unsigned int i = 0 ; i < dim ; ++i )
				delete[] mp_Solution[i];
			delete[] mp_Solution;
			mp_Solution = 0;
		}
	}

	//===============================================================================================



	/// 前處理先將要解的矩陣分解成 LLT.
	/// note : 對矩陣 A 做 LLT factorize.
	bool GeneralSLSManager::PreprocessLLTFactorization()
	{
		assert( mp_LLTFactor == 0 );

		//taucs_ccs_matrix* A = mp_LinearSystem->GetA()->GetMatrix();

		// factorize the matrix A.
		//int error = Taucs::LinSolveLLTFactor( A, mp_LLTFactor );
		int error = mp_LinearSystem->GetA()->LLTFactorize( mp_LLTFactor );

		if( mp_LLTFactor == 0 )
		{
			Taucs::ErrorReport( error );	
			return false;
		}
		return true;			
	}

	/// 解線性系統.
	bool GeneralSLSManager::Solve()
	{
		try
		{
			if( mp_LinearSystem == 0 )
				return false;

			// check matrix A. it should be a symmetric matrix.
			if( !mp_LinearSystem->GetA()->IsSymmetric() )
				throw exception("Matrix A should be symmetric!!");

			// do factorize matrix A if it don't exist
			if( mp_LLTFactor == 0 )
				if( PreprocessLLTFactorization() == false )
					return false;

			assert( mp_LLTFactor );

			// solution memory 應該在之前就已經 allocate 好了。
			assert( mp_Solution );

			// solve the linear system.
			//return ((GeneralSparseLSSolver*)GeneralSparseLSSolver::GetInstance())->Solve( 
			//	mp_LinearSystem, mp_LLTFactor, mp_Solution );	
			return ((GeneralSparseLSSolver*)GeneralSparseLSSolver::GetInstance())->Solve( 
				mp_LinearSystem, (const void *&)mp_LLTFactor, mp_Solution );	
		}
		catch( const exception& e )
		{
			throw e;
		}
	}

	//===============================================================================================


	/// pLinearSystem 將由此物件負責刪除.
	/// pLinearSystem : a A*x = B linear system.
	LeastSquareSLSManager::LeastSquareSLSManager( SparseLinearSystem* pLinearSystem )
		: SparseLinearSystemManager( pLinearSystem )
		, mp_ATranspose( 0 )
		, mp_NormalEquationMatrix( 0 )
		, mi_Dim( 0 )
		, mp_AtB( 0 )
	{
		if( pLinearSystem )
		{
			_update_data( pLinearSystem );
		}
	}

	void LeastSquareSLSManager::_free_memory()
	{
		delete mp_ATranspose; 
		mp_ATranspose = 0;

		delete mp_NormalEquationMatrix; 
		mp_NormalEquationMatrix = 0;

		Delete2DArray( mp_AtB, mi_Dim );
		mi_Dim = 0;
	}

	void LeastSquareSLSManager::_update_data( SparseLinearSystem* pLS )
	{
		assert( pLS );

		const StableSparseMatrix* A = pLS->GetA();

		mp_ATranspose = (StableSparseMatrix*) A->Transpose();
		mp_NormalEquationMatrix = mp_ATranspose->MultiplyNonSymmMatSymmResult(*A);

		// check matrix At*A. it should be a symmetric matrix.
		if( !mp_NormalEquationMatrix->IsSymmetric() )
			throw exception("Matrix At*A is not a symmetric matrix!!");

		// get the dimension of this linear system.
		mi_Dim = (int) pLS->Dimension();

		unsigned int n = A->GetNumCols();

		mp_AtB = Allocate2DArray<double>( mi_Dim, n );
	}

	/// pLinearSystem 將由此物件負責刪除.
	/// pLinearSystem 不可以為 NULL.
	void LeastSquareSLSManager::SetLinearSystem( SparseLinearSystem* pLinearSystem )
	{
		SparseLinearSystemManager::SetLinearSystem( pLinearSystem );

		_free_memory();
		_update_data( pLinearSystem );
	}

	/// 前處理先將要解的矩陣分解成 LLT.
	/// note : 對矩陣 A 做 LLT factorize.
	bool LeastSquareSLSManager::PreprocessLLTFactorization()
	{
		assert( mp_LLTFactor == 0 );

		// factorize the matrix (At*A).
		int error = mp_NormalEquationMatrix->LLTFactorize( mp_LLTFactor );

		if( mp_LLTFactor == 0 )
		{
			Taucs::ErrorReport( error );	
			return false;
		}

		return true;
	}

	/// 解線性系統.
	bool LeastSquareSLSManager::Solve()
	{
		if( mp_LinearSystem == 0 )
			return false;

		try
		{
			// solve linear system.
			//--------------------------------------------------------

			// do factorize matrix A if it don't exist
			if( mp_LLTFactor == 0 )
				if( PreprocessLLTFactorization() == false )
					return false;

			// solution memory 應該在之前就已經 allocate 好了。
			assert( mp_LLTFactor && mp_Solution );

			double** &B = mp_LinearSystem->GetB();
			for( int i = 0 ; i < mi_Dim ; ++i )
				mp_ATranspose->MultiplyVector( B[i], mp_AtB[i] );

			// solve the linear system.
			//return ((LeastSquareSparseLSSolver*) LeastSquareSparseLSSolver::GetInstance())->Solve(
			//	mp_NormalEquationMatrix, (const double**)mp_AtB, mi_Dim, mp_LLTFactor, mp_Solution );
			return ((LeastSquareSparseLSSolver*) LeastSquareSparseLSSolver::GetInstance())->Solve(
				mp_NormalEquationMatrix, (const double**)mp_AtB, mi_Dim, (const void *&)mp_LLTFactor, mp_Solution );
		}
		catch( const exception& e )
		{
			throw e;
		}
	}
}