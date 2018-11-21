#ifndef _LINEARSYSTEMLIB_SPARSE_LINEAR_SYSTEM_MANAGER_H_
#define _LINEARSYSTEMLIB_SPARSE_LINEAR_SYSTEM_MANAGER_H_

#include "SparseLinearSystem.h"
#include "TaucsWraper.h"

namespace LinearSystemLib
{
	class StableSparseMatrix;

	/*!
		\note
		�Φb�� sparse linear system �ɡA�D�x�} A �����ܪ����p�U�AB �i��@��
		�b�ܰʡA�����p�U�A�i�H�w���� matrix A ���e�B�z���Ѧ� LLT
		�Ψӥ[�t�� sparse linear system, �t�~�]�t�d�޲z linear system
		�٦��i�H�D�ѵ�����.
	*/
	class SparseLinearSystemManager
	{
	protected:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem �N�Ѧ�����t�d�R��. 
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

		/// pLinearSystem �N�Ѧ�����t�d�R��.
		/// pLinearSystem ���i�H�� NULL.
		virtual void SetLinearSystem( SparseLinearSystem* pLinearSystem );

		/// �e�B�z���N�n�Ѫ��x�}���Ѧ� LLT.
		virtual bool PreprocessLLTFactorization() = 0;

		/// �ѽu�ʨt��.
		virtual bool Solve() = 0;

		/// �M�Ť����Ҧ� member data
		virtual void Reset();

	protected:

		void freeMemory();

		void freeLLTFactorization();

	private:

		void allocate_solution( unsigned int dim1, unsigned int dim2 );

		void free_solution( unsigned int dim );

	protected:

		/// �n�Ѫ��u�N�t�ΡA�~���i�H�ק�ȡA�����i�H�ק� dimension �M
		/// �n�Ѫ��ӼơC�Y���o�ض��D�A�����ϥ� SetLinearSystem method.
		SparseLinearSystem* mp_LinearSystem;

		/// store the main solve linear system's matrix factorization matrix.
		void*		mp_LLTFactor;

		/// �t�Ϊ���.
		double**	mp_Solution;
	};

	/// pLinearSystem �N�Ѧ�����t�d�R��.
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

	/// �t�d general sparse linear system �� class.
	class GeneralSLSManager : public SparseLinearSystemManager
	{

	private:
		GeneralSLSManager( const GeneralSLSManager& );
		GeneralSLSManager& operator= ( const GeneralSLSManager& );

	public:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem �N�Ѧ�����t�d�R��. 
		*/
		GeneralSLSManager( SparseLinearSystem* pLinearSystem = 0 )
			: SparseLinearSystemManager( pLinearSystem )
		{
			;
		}

	public:

		//! �N�n�Ѫ��x�}���Ѧ� LLT.
		/*! \note note : ��x�} A �� LLT factorize. */
		virtual bool PreprocessLLTFactorization();

		//! solve the Ax=B solution of this linear system.
		virtual bool Solve();
	};

	//===============================================================================================

	/// �t�d least square sparse linear system �� class.
	class LeastSquareSLSManager : public SparseLinearSystemManager
	{
	private:
		LeastSquareSLSManager( const LeastSquareSLSManager& );
		LeastSquareSLSManager& operator= ( const LeastSquareSLSManager& );

	public:

		//! constructor.
		/*! 
			\param pLinearSystem : a A*x = B linear system.
			\note pLinearSystem �N�Ѧ�����t�d�R��. 
		*/
		LeastSquareSLSManager( SparseLinearSystem* pLinearSystem = 0 );

		virtual ~LeastSquareSLSManager() { _free_memory(); }

	public:

		/*! \note pLinearSystem �N�Ѧ�����t�d�R�� �B pLinearSystem ���i�H�� NULL. */
		virtual void SetLinearSystem( SparseLinearSystem* pLinearSystem );

		//! �N�n�Ѫ��x�}���Ѧ� LLT.
		/*! \note note : ��x�} AtA �� LLT factorize. */
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