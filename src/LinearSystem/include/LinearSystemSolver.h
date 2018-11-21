#ifndef _LINEAR_SYSTEM_SOLVER_H_
#define _LINEAR_SYSTEM_SOLVER_H_

#include "Prerequistites.h"
#include "linearsystem.h"

namespace LinearSystemLib
{
	class SparseLinearSystem;
	class StableSparseMatrix;

	_interface LinearSystemSolver
	{
		virtual ~LinearSystemSolver() {;}

		/// _out_sol : 不須要先 allocate，this method will allocate enough memory.
		virtual bool Solve( const LinearSystem* _system, double** &_out_sol ) const = 0;
	};

	/// 用來解 A * x = B
	class GeneralSparseLSSolver 
		: public LinearSystemSolver
	{
	private:
		GeneralSparseLSSolver() {}
		GeneralSparseLSSolver( const GeneralSparseLSSolver& );
		GeneralSparseLSSolver& operator= ( const GeneralSparseLSSolver& );

	public:

		static LinearSystemSolver* GetInstance();

		//! solve a A*x = B solution.
		/*!
			\param _system the linear system want to be solve, 
						   it's type must be the SparseLinearSystem type; it is a A*x = B.
			\param _out_sol the solution will be stored here.
			\note _out_sol 不須要先 allocate，this method will allocate enough memory.
		*/
		virtual bool Solve( const LinearSystem* _system, double** &_out_sol ) const;

		/// note : the _out_sol 要先配置好足夠的空間.
		virtual bool Solve( const SparseLinearSystem* _system, const void*& _factorization, double** &_out_sol ) const;
	};

	inline LinearSystemSolver* GeneralSparseLSSolver::GetInstance()
	{
		static GeneralSparseLSSolver _instance;
		return &_instance;
	}

	//=============================================================================================

	/// 用來解 (A^t * A) * x = (A^t * B)
	class LeastSquareSparseLSSolver 
		: public LinearSystemSolver
	{
	private:
		LeastSquareSparseLSSolver() {}
		LeastSquareSparseLSSolver( const LeastSquareSparseLSSolver& );
		LeastSquareSparseLSSolver& operator= ( const LeastSquareSparseLSSolver& );

	public:		

		//! get the singleton object of this class.
		static LinearSystemSolver* GetInstance();

		//! solve a At*A*x = At*B solution.
		/*!
		    \param _system the linear system want to be solve, 
						   it's type must be the SparseLinearSystem type; it is a A*x = B.
			\param _out_sol the solution will be stored here.
			\note _out_sol 不須要先 allocate，this method will allocate enough memory.
		*/
		virtual bool Solve( const LinearSystem* _system, double** &_out_sol ) const;

		/// _system : a A*x = B linear system.
		/// _factorization 是 _system 裡 matrix A 的 ( At * A ) 的 llt 分解矩陣
		/// _A_transpose : _system's matrix A's transpose matrix.
		/// note : the _out_sol 要先配置好足夠的空間.
		// virtual bool Solve( const SparseLinearSystem* _system, const void*& _factorization, 
		//	const StableSparseMatrix* _At_A, const StableSparseMatrix* _A_transpose, 
		//	double** &_out_sol ) const;

		//! solve a At*A*x = At*B solution.
		/*!
			\param AtA the normal equation of matrix A.
			\param AtB the result of At * B.
			\param dim   the dimension of this linear system.
			\param factor the factorization matrix of AtA.
			\param out_sol the solved solution will be stored here.

			\note all parameters can't be NULL.
			      out_sol 要先配置好足夠的空間.
		*/
		virtual bool Solve( const StableSparseMatrix* AtA, const double** AtB, int dim,
			const void*& factor, double** &out_sol ) const;
	};

	inline LinearSystemSolver* LeastSquareSparseLSSolver::GetInstance()
	{
		static LeastSquareSparseLSSolver _instance;
		return &_instance;

	}
	//=============================================================================================
}

#endif