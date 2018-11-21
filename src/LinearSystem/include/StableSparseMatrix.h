#ifndef _LINEARSYSTEMLIB_STABLE_SPARSE_MATRIX_H_
#define _LINEARSYSTEMLIB_STABLE_SPARSE_MATRIX_H_

#include <iostream>
#include <vector>
#include <map>

#include "taucs_all.h"

#include "Matrix.h"
#include "GeneralSparseMatrix.h"

namespace LinearSystemLib
{
	using std::ostream;
	using std::map;
	using std::vector;

	// ps. 目前只適用於 非 symmetric 的 object, 在這種情況下，normalequationform 才會是對的.
	class StableSparseMatrix : public SparseMatrix
	{
	public:
		StableSparseMatrix();
		StableSparseMatrix( const taucs_ccs_matrix* _tcm );
		StableSparseMatrix( const vector< map<unsigned int,double>* >& cols, int nRows, int flags ); 
		StableSparseMatrix( const StableSparseMatrix& _tsm );
		StableSparseMatrix( const GeneralSparseMatrix& _sm );

		virtual ~StableSparseMatrix();

		StableSparseMatrix& operator = ( const StableSparseMatrix& _tsm );
		StableSparseMatrix& operator = ( const GeneralSparseMatrix& _sm );

	public:

		// clone this object.
		virtual Matrix* Clone() const;

		bool Create( const vector< map<unsigned int,double>* >& cols, int nRows, int flags );

		/// set the native matrix to this matrix.
		/// note: pMat 的 memory 會交由這個類別管理.
		void Set( taucs_ccs_matrix* pMat );

		/// reset the matrix's data.
		void Free();

		// is this matrix symmetric.
		bool IsSymmetric() const;

		// note : 自己不能是 symmetric.
		StableSparseMatrix* MultiplyNonSymMatrix( const StableSparseMatrix& _ssm ) const;

		// for usage when it's known that the result is symmetric,
		// like A^T * A
		StableSparseMatrix* MultiplyNonSymmMatSymmResult( const StableSparseMatrix& _ssm ) const;

		int Flags() const;

		taucs_ccs_matrix* GetMatrix();
		const taucs_ccs_matrix* GetMatrix() const;

		/// 做 LLT factorize.
		int LLTFactorize( void*& out_l ) const;

	public: // overriding functions.

		/// transpose matrix.
		Matrix* Transpose() const;

		double GetElement( unsigned int _row_index, unsigned int _col_index ) const;

		// get the normal equation form, if this matrix is A, it will return A^T * A.
		// note : 自己不能是 symmetric.
		SparseMatrix* NormalEquationForm() const;

		// Multiplies this matrix by x and stores the result in b. Assumes all memory has been allocated
		// and the sizes match; assumes this matrix is not symmetric!!
		void MultiplyVector( double* _x, double* _b ) const;

		void MultiplyVector( const vector<double>& _x, double* _b ) const;

		//void Write( const string& _filename, ISerializeSparseMatrix* _format ) const;
		void Write( ostream& _out, SerializeMatrix* _format ) const;

		unsigned int GetNumRows() const;
		unsigned int GetNumCols() const;
		unsigned int GetNumNonZero() const;

		virtual SerializeMatrix* GetSerializeIjv() const;
		virtual SerializeMatrix* GetSerializeFull() const;

	private:
		taucs_ccs_matrix* m_pTaucsMatrix;
	};
}

#endif