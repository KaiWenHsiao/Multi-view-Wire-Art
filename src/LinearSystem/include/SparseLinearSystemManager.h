#ifndef _LINEARSYSTEMLIB_SPARSE_LINEAR_SYSTEM_MANAGER_H_
#define _LINEARSYSTEMLIB_SPARSE_LINEAR_SYSTEM_MANAGER_H_

#include "SparseLinearSystem.h"
#include "TaucsWraper.h"

namespace LinearSystemLib
{
	class StableSparseMatrix;

	/*!
		\note
		用在解 sparse linear system 時，主矩陣 A 不改變的情況下，B 可能一直
		在變動，此情況下，可以預先對 matrix A 做前處理分解成 LLT
		用來加速解 sparse linear system, 另外也負責管理 linear system
		還有可以求解等介面.
	*/
	class SparseLinearSystemManager
	{
	protected:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem 將由此物件負責刪除. 
		*/
		SparseLinearSystemManager( SparseLinearSystem* pLinearSystem = 0 );

	public:

		virtual ~SparseLinearSystemManager()	{ freeMemory(); }

		/// return the linear system's dimension
		unsigned int Dimension() const			{ return mp_LinearSystem->Dimension(); }

		/// return the solution value.
		double** GetSolution() 					{ return mp_Solution; }

		/// return current linear system.
		SparseLinearSystem* GetLinearSystem()	{ return mp_LinearSystem; }

	public:

		/// pLinearSystem 將由此物件負責刪除.
		/// pLinearSystem 不可以為 NULL.
		virtual void SetLinearSystem( SparseLinearSystem* pLinearSystem );

		/// 前處理先將要解的矩陣分解成 LLT.
		virtual bool PreprocessLLTFactorization() = 0;

		/// 解線性系統.
		virtual bool Solve() = 0;

		/// 清空內部所有 member data
		virtual void Reset();

	protected:

		void freeMemory();

		void freeLLTFactorization();

	private:

		void allocate_solution( unsigned int dim1, unsigned int dim2 );

		void free_solution( unsigned int dim );

	protected:

		/// 要解的線代系統，外面可以修改值，但不可以修改 dimension 和
		/// 要解的個數。若有這種須求，必須使用 SetLinearSystem method.
		SparseLinearSystem* mp_LinearSystem;

		/// store the main solve linear system's matrix factorization matrix.
		void*		mp_LLTFactor;

		/// 系統的解.
		double**	mp_Solution;
	};

	/// pLinearSystem 將由此物件負責刪除.
	inline SparseLinearSystemManager::SparseLinearSystemManager( SparseLinearSystem* pLinearSystem )
		: mp_LinearSystem( pLinearSystem )
		, mp_LLTFactor( 0 )
		, mp_Solution( 0 )
	{
		if( mp_LinearSystem )
			allocate_solution( mp_LinearSystem->Dimension(), mp_LinearSystem->GetA()->GetNumCols() );
	}

	inline void SparseLinearSystemManager::Reset()
	{
		freeMemory();
	}

	inline void SparseLinearSystemManager::freeLLTFactorization()
	{
		if( mp_LLTFactor )
		{
			TaucsLib::Taucs::LinSolveFree( mp_LLTFactor );
			mp_LLTFactor = 0;
		}
	}

	//===============================================================================================

	/// 負責 general sparse linear system 的 class.
	class GeneralSLSManager : public SparseLinearSystemManager
	{

	private:
		GeneralSLSManager( const GeneralSLSManager& );
		GeneralSLSManager& operator= ( const GeneralSLSManager& );

	public:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem 將由此物件負責刪除. 
		*/
		GeneralSLSManager( SparseLinearSystem* pLinearSystem = 0 )
			: SparseLinearSystemManager( pLinearSystem )
		{
			;
		}

	public:

		//! 將要解的矩陣分解成 LLT.
		/*! \note note : 對矩陣 A 做 LLT factorize. */
		virtual bool PreprocessLLTFactorization();

		//! solve the Ax=B solution of this linear system.
		virtual bool Solve();
	};

	//===============================================================================================

	/// 負責 least square sparse linear system 的 class.
	class LeastSquareSLSManager : public SparseLinearSystemManager
	{
	private:
		LeastSquareSLSManager( const LeastSquareSLSManager& );
		LeastSquareSLSManager& operator= ( const LeastSquareSLSManager& );

	public:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem 將由此物件負責刪除. 
		*/
		LeastSquareSLSManager( SparseLinearSystem* pLinearSystem = 0 );

		virtual ~LeastSquareSLSManager() { _free_memory(); }

	public:

		/*! \note pLinearSystem 將由此物件負責刪除 且 pLinearSystem 不可以為 NULL. */
		virtual void SetLinearSystem( SparseLinearSystem* pLinearSystem );

		//! 將要解的矩陣分解成 LLT.
		/*! \note note : 對矩陣 AtA 做 LLT factorize. */
		virtual bool PreprocessLLTFactorization();

		//! solve the least square solution of this linear system.
		virtual bool Solve();

		virtual void Reset();

	private:

		void _free_memory();
		void _update_data( SparseLinearSystem* pLinearSystem );

	protected:
		
		/// store the matrix A's transpose.
		StableSparseMatrix*	mp_ATranspose;

		/// store the matrix A's (At * A) matrix.
		StableSparseMatrix* mp_NormalEquationMatrix;

		/// store the dimension of the linear system.
		int mi_Dim;

		/// pre-allocate the right matrix AtB memory.
		double** mp_AtB;
	};

	inline void LeastSquareSLSManager::Reset()
	{
		SparseLinearSystemManager::Reset();
		_free_memory();
	}
}

#endif