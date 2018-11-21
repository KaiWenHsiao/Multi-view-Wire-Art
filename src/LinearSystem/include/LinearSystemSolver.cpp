#include "linearsystemsolver.h"

#include "TaucsWraper.h"
#include "SparseLinearSystem.h"
#include "GeneralSparseMatrix.h"
#include "StableSparseMatrix.h"

#include <fstream>
#include <cassert>
#include <memory>

#include "MemoryUtils.h"

namespace LinearSystemLib
{
	using namespace std;
	using namespace TaucsLib;

	//=============================================================================================================


	// 第一個參數必須要是 SparseLinearSystem 型別的.
	bool GeneralSparseLSSolver::Solve( const LinearSystem* _system, double** &_out_sol ) const
	{
		try
		{
			SparseLinearSystem* sls = const_cast<SparseLinearSystem*>( (const SparseLinearSystem*)_system );

			if( sls == NULL )
				throw exception("GeneralSparseLSSolver::Solve : first param must be the SparseLinearSystem type.");

			// 先取得 Matrix A and Matrix B.
			taucs_ccs_matrix* A = sls->GetA()->GetMatrix();
			assert( A );

			if( !sls->GetA()->IsSymmetric() )
				throw exception("Matrix A should be symmetric!!");

			double** B = sls->GetB();

			// get the dimension of matrix A.
			const unsigned int n = sls->GetA()->GetNumCols();

			// get the matrix B's dimension.
			const unsigned int d = sls->Dimension();

			//--------------------------------------------------------
			// prepare and initialize the solutions.
			//--------------------------------------------------------

			_out_sol = Allocate2DArray<double>( d, n );

			//--------------------------------------------------------
			// solve the linear system equation.
			//--------------------------------------------------------

			bool success = true;

			//---------------------------------------------------------
			// here is the way 1.
			// solver way 1.
			// this way is very slowly, depend on second params.

			//taucs_ccs_matrix* L = Taucs::FactorToLLT( A, 0.0, 0 );

			//if( L == NULL )
			//{
			//	success = false;			
			//	throw exception("?");
			//}
			//else
			//{
			//	for( int i = 0 ; success && i < d ; ++i )
			//		if( Taucs::SolveLLT( L, _out_sol[i], B[i] ) != TAUCS_SUCCESS ) 
			//			success = false;
			//	Taucs::Free(L);
			//}

			//---------------------------------------------------------
			// solve way 2.
			//----------------------------------------------------
			// solve algorithm :
			//	step1 : 先對 A 做 Factorization 並存在 F.
			//	step2 : 利用 F 來解多次 linear system.
			//	step3 : 釋放 factorization F.
			//----------------------------------------------------

			void* F = NULL;

			// step1.
			int error = Taucs::LinSolveLLTFactor( A, F );

			if( F == NULL )
			{
				success = false;
				Taucs::ErrorReport( error );			
				throw exception("LinSolveLLTFactor Failed in GeneralSparseLSSolver::Solve");
			}
			else
			{			
				// 因為是用 分解的方式做，因此須要 solve_opt.
				char* solve_opt [] = { "taucs.factor=false", NULL };

				// step2.
				for( unsigned int i = 0 ; success && i < d ; ++i )
					if( Taucs::LinSolve( A, &F, 1, _out_sol[i], B[i], solve_opt, NULL ) != TAUCS_SUCCESS ) 
						success = false;
			}

			// step3.
			Taucs::LinSolveFree( F );

			// way 3
			//char* options[] = {"taucs.factor.LLT=true", NULL };

			//for( int i = 0 ; i < d ; ++i )
			//	if( TAUCS_SUCCESS != Taucs::LinSolve( A, NULL, 1, _out_sol[i], B[i], options, NULL) )
			//		return false;

			return success;		
		}
		catch( const exception& e )
		{
			throw e; //exception("GeneralSparseLSSolver::Solve : first param must be the SparseLinearSystem type.");
		}
	}

	bool GeneralSparseLSSolver::Solve( const SparseLinearSystem* _system, const void*& _factorization, double** &_out_sol ) const
	{
		try
		{
			if( _system == 0 || _factorization == 0 )
				return false;

			SparseLinearSystem* sls = const_cast<SparseLinearSystem*>( _system );
			void*& factor = const_cast< void*& >( _factorization );

			// 先取得 Matrix A and Matrix B.
			taucs_ccs_matrix* A = sls->GetA()->GetMatrix();
			double** B = sls->GetB();

			// check matrix A, it should be a symmetric matrix.
			if( !sls->GetA()->IsSymmetric() )
				throw exception("Matrix A should be symmetric!!");

			//--------------------------------------------------------
			// solve the linear system equation.
			//--------------------------------------------------------

			// 因為是用 分解的方式做，因此須要 solve_opt.
			char* solve_opt [] = { "taucs.factor=false", NULL };

			//	利用 factorization 來解多次 linear system.
			unsigned int d = sls->Dimension();
			for( unsigned int i = 0 ; i < d ; ++i )
				if( Taucs::LinSolve( A, &factor, 1, _out_sol[i], B[i], solve_opt, NULL ) != TAUCS_SUCCESS ) 
					return false;

			return true;		
		}
		catch( const exception& e )
		{
			throw e;
		}
	}

	//=============================================================================================================


	// 第一個參數必須要是 SparseLinearSystem 型別的.
	bool LeastSquareSparseLSSolver::Solve( const LinearSystem* _system, double** &_out_sol ) const
	{		
		try
		{
			SparseLinearSystem* sls = const_cast<SparseLinearSystem*>( (const SparseLinearSystem*) _system );

			if( sls == NULL )
				throw exception("GeneralSparseLSSolver::Solve : first param must be the SparseLinearSystem type.");

			// 先取得 Matrix A and Matrix B.
			StableSparseMatrix* A = sls->GetA();
			assert(A); 

			// 計算 At * A.
			auto_ptr<StableSparseMatrix> At( (StableSparseMatrix*) A->Transpose() );
			assert( At.get() ); 

			auto_ptr<StableSparseMatrix> AtA( (StableSparseMatrix*) At->MultiplyNonSymmMatSymmResult(*A) );
			assert( AtA.get() ); 

			// 計算 At * B.		
			//--------------------------------------------------------

			// get the dimension of matrix A.
			const unsigned int n = A->GetNumCols();

			// get matrix B.
			double** B = sls->GetB();

			// get the matrix B's dimension.
			const unsigned int d = sls->Dimension();

			double** AtB = new double* [d];

			for( unsigned int i = 0 ; i < d ; ++i )
			{
				AtB[i] = new double[n];
				At->MultiplyVector( &B[i][0], AtB[i] );
			}

			// solve the linear system equation.
			//--------------------------------------------------------

			SparseLinearSystem new_sls( AtA.release(), AtB, d );
			bool flag = GeneralSparseLSSolver::GetInstance()->Solve( &new_sls, _out_sol );

			return flag;

		}
		catch( const exception& e )
		{
			throw e; //exception("GeneralSparseLSSolver::Solve : first param must be the SparseLinearSystem type.");
		}
	}

	//bool LeastSquareSparseLSSolver::Solve( const SparseLinearSystem* _system, const void*& _factorization, 
	//		const StableSparseMatrix* _At_A, const StableSparseMatrix* _A_transpose, double** &_out_sol ) const
	//{

	//	if( _system == 0 || _factorization == 0 || _A_transpose == 0 )
	//		return false;

	//	SparseLinearSystem* sls = const_cast<SparseLinearSystem*>( _system );
	//	void*& factor = const_cast< void*& >( _factorization );

	//	// 先取得 Matrix A and Matrix B.
	//	StableSparseMatrix* A = sls->GetA();

	//	// get matrix B.
	//	double** B = sls->GetB();			

	//	taucs_ccs_matrix* ATA = const_cast<taucs_ccs_matrix*>( _At_A->GetMatrix() );

	//	// 計算 At * B.		
	//	//--------------------------------------------------------

	//	// get the dimension of matrix A.
	//	unsigned int n = A->GetNumCols();

	//	// get the matrix B's dimension.
	//	int d = (int) sls->Dimension();

	//	double** AtB = new double* [d];

	//	for( int i = 0 ; i < d ; ++i )
	//	{
	//		AtB[i] = new double[n];
	//		_A_transpose->MultiplyVector( &B[i][0], AtB[i] );
	//	}

	//	bool success = true;

	//	try
	//	{
	//		// solve the linear system equation.
	//		//--------------------------------------------------------

	//		// 因為是用 分解的方式做，因此須要 solve_opt.
	//		char* solve_opt [] = { "taucs.factor=false", NULL };

	//		//	利用 factorization 來解多次 linear system.
	//		
	//		for( int i = 0 ; i < d && success ; ++i )
	//			if( Taucs::LinSolve( ATA, &factor, 1, _out_sol[i], AtB[i], solve_opt, NULL ) != TAUCS_SUCCESS ) 
	//				success = false;
	//	}
	//	catch( const exception& e )
	//	{
	//		throw e;
	//	}			
	//	
	//	Delete2DArray( AtB, d );

	//	return success;
	//}


	bool LeastSquareSparseLSSolver::Solve( 
		const StableSparseMatrix* AtA, const double** AtB, int dim,
		const void*& factor, double** &out_sol ) const
	{

		if( AtA == 0 || AtB == 0 || factor == 0 || out_sol == 0 )
			return false;

		void*& fac = const_cast< void*& >( factor );

		taucs_ccs_matrix* ata = const_cast<taucs_ccs_matrix*>( AtA->GetMatrix() );
		double** atb = (double**) AtB;

		// start to solve the linear system equation.			
		
		// 因為是用 分解的方式做，因此須要 solve_opt.
		char* solve_opt [] = { "taucs.factor=false", NULL };

		bool success = true;

		try
		{
			//	利用 factorization 來解多次 linear system.
			for( int i = 0 ; i < dim && success ; ++i )
				if( Taucs::LinSolve( ata, &fac, 1, out_sol[i], atb[i], solve_opt, NULL ) != TAUCS_SUCCESS ) 
					success = false;
		}
		catch( const exception& e )
		{
			throw e;
		}

		return success;
	}
}