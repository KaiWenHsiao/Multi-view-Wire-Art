#ifndef _LINEARSYSTEMLIB_GENERAL_SPARSE_MATRIX_H_
#define _LINEARSYSTEMLIB_GENERAL_SPARSE_MATRIX_H_

#include "Matrix.h"

#include <vector>
#include <map>
#include <memory>
#include <cassert>

namespace LinearSystemLib
{
	using namespace std;

	/// class GeneralSparseMatrix
	/// this class is for dynamic sparse matrix. 
	/// that is, you can set, delete rows, delete cols.
	/// the matrix is not static, it will be changed if you want.

	 // class GeneralSymmetricSpraseMatrix; ...

	class GeneralSparseMatrix : public SparseMatrix
	{
	public:
		GeneralSparseMatrix();

		GeneralSparseMatrix( const GeneralSparseMatrix& _sm ); 

		GeneralSparseMatrix( unsigned int _rows, unsigned int _cols );

		virtual ~GeneralSparseMatrix();

		GeneralSparseMatrix& operator = ( const GeneralSparseMatrix& _sm );

	public:

		void SetElement( unsigned int _row_index, unsigned int _col_index, double _value );

		void Free();

		void Create( unsigned int _rows, unsigned int _cols );

		void DeleteRows( unsigned int _first, unsigned int _last );
		void DeleteCols( unsigned int _first, unsigned int _last );

		/// 將 _first, _last 這之間的 column 刪去, 並存入 _dest.
		/// push_back into the _dest.
		template < class _OIter >
		void DeleteColsStoreTo( unsigned int _first, unsigned int _last, 
			 _OIter _dest_beg )
		{
			assert( 0 <= _first && _first < GetNumCols() && 0 <= _last && 
				_last <= GetNumCols() && _last > _first );

			vector< map<unsigned int, double>* >::iterator begin = m_cElements.begin();
			vector< map<unsigned int, double>* >::iterator end = begin;

			advance( begin, _first );
			advance( end, _last );

			// assign the pointer to _dest_beg.
			while( _first != _last )
			{
				*_dest_beg = m_cElements[_first];
				++_dest_beg;
				++_first;
			}

			// 單純的 erase, 不要 delete memory.
			m_cElements.erase( begin, end );
		}

		/// 將 _first, _last 這之間的 column 刪去, 並存入 _dest.
		/// push_back into the _dest.
		void DeleteColsStoreTo( unsigned int _first, unsigned int _last, 
			vector< map<unsigned int,double>* >& _dest );

		GeneralSparseMatrix* Multiply( const GeneralSparseMatrix& _sm ) const;

		vector< map<unsigned int,double>* >& Elements()				{ return m_cElements; }
		const vector< map<unsigned int,double>* >& Elements() const { return m_cElements; }
		
		/// 取得第 ith 行的 data.
		const map<unsigned int, double>& GetColumn( unsigned int _ith ) const;
		/// 取得第 ith 行的 data.
		map<unsigned int, double>& GetColumn( unsigned int _ith );

		bool IsSymmetric() const;

	public: // overriding functions.

		virtual Matrix* Clone() const;

		virtual Matrix* Transpose() const;

		virtual double GetElement( unsigned int _row_index, unsigned int _col_index ) const;

		// get the normal equation form, if this matrix is A, it will return A^T * A.
		virtual SparseMatrix* NormalEquationForm() const;

		// Multiplies this matrix by x and stores the result in b. Assumes all memory has been allocated
		// and the sizes match; assumes this matrix is not symmetric!!
		virtual void MultiplyVector( double* _x, double* _b ) const;

		virtual void Write( ostream& _out, SerializeMatrix* _format ) const;

		virtual unsigned int GetNumRows() const		{ return m_uRows; }
		virtual unsigned int GetNumCols() const		{ return static_cast<unsigned int>( m_cElements.size() ); }
		virtual unsigned int GetNumNonZero() const;

		virtual SerializeMatrix* GetSerializeIjv() const;
		virtual SerializeMatrix* GetSerializeFull() const;

	protected:

		// store the number of rows.
		unsigned int m_uRows;

		// 用 compress column storage 的方式來存
		// 在此 vector 用來存 column ptr, 而 map 用來存 (key, value) => (row index, element value)
		vector< map<unsigned int, double>* > m_cElements;
	};

	inline GeneralSparseMatrix::GeneralSparseMatrix()
	{
		m_uRows = 0;
	}

	inline GeneralSparseMatrix::GeneralSparseMatrix( const GeneralSparseMatrix& _sm )
		: SparseMatrix(_sm)
	{
		*this = _sm;
	}

	inline GeneralSparseMatrix::~GeneralSparseMatrix()
	{
		Free();
	}

	inline Matrix* GeneralSparseMatrix::Clone() const
	{
		return new GeneralSparseMatrix(*this);
	}

	inline double GeneralSparseMatrix::GetElement( unsigned int _row_index, unsigned int _col_index ) const
	{
		assert( 0 <= _row_index && _row_index < m_uRows && 0 <= _col_index && _col_index < GetNumCols() );

		map<unsigned int, double>::const_iterator cit = m_cElements[_col_index]->find(_row_index);
		return (cit == m_cElements[_col_index]->end() ? 0.0 : cit->second );
	}

	/// 取得第 ith 行的 data.
	inline const map<unsigned int, double>& GeneralSparseMatrix::GetColumn( unsigned int _ith ) const
	{
		assert( 0 <= _ith && _ith < m_cElements.size() );
		return *m_cElements[_ith];
	}

	/// 取得第 ith 行的 data.
	inline map<unsigned int, double>& GeneralSparseMatrix::GetColumn( unsigned int _ith )
	{
		assert( 0 <= _ith && _ith < m_cElements.size() );
		return *m_cElements[_ith];
	}

	// get the normal equation form, if this matrix is A, it will return A^T * A.
	inline SparseMatrix* GeneralSparseMatrix::NormalEquationForm() const
	{
		auto_ptr<GeneralSparseMatrix> transpose( static_cast<GeneralSparseMatrix*>( Transpose() ) );
		//GeneralSparseMatrix* transpose = static_cast<GeneralSparseMatrix*>( Transpose() );
		GeneralSparseMatrix* result = static_cast<GeneralSparseMatrix*>( transpose->Multiply(*this) );
		//delete transpose;
		return result;
	}

	inline void GeneralSparseMatrix::Write( ostream& _out, SerializeMatrix* _format ) const
	{
		assert( _format );
		_format->Write( _out, this );
	}

	inline unsigned int GeneralSparseMatrix::GetNumNonZero() const
	{
		unsigned int nnz = 0u;
		for( unsigned int i = 0 ; i < m_cElements.size() ; ++i )
			nnz += static_cast<unsigned int>(m_cElements[i]->size());
		return nnz;
	}

}

#endif