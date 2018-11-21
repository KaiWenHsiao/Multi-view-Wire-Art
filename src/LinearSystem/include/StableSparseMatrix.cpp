#include "StableSparseMatrix.h"

#include <algorithm>
#include <cassert>
#include "TaucsWraper.h"
#include "Singleton.h"

using namespace std;

namespace LinearSystemLib
{
	using namespace TaucsLib;

	//========================================================================================================
	namespace 
	{
		class SerializeStableSparseMatrixIjv 
			: public SerializeMatrix
			, public Singleton<SerializeStableSparseMatrixIjv>
		{
		public:
			// 第二個必須是可以轉型成 StableSparseMatrix，因為是 for StableSparseMatrix.
			virtual void Write( ostream& _out, const Matrix* _sm ) const;
		};

		class SerializeStableSparseMatrixFull 
			: public SerializeMatrix
			, public Singleton<SerializeStableSparseMatrixFull>
		{
		public:
			// 第二個必須是可以轉型成 StableSparseMatrix，因為是 for StableSparseMatrix
			virtual void Write( ostream& _out, const Matrix* _sm ) const;

		private:
			void _write_symm_taucs_lower_( ostream& _out, const StableSparseMatrix* _sm ) const;
			void _write_symm_taucs_upper_( ostream& _out, const StableSparseMatrix* _sm ) const;
		};

		//-----------------------------------------------------------------------

		void SerializeStableSparseMatrixIjv::Write( ostream& _out, const Matrix* _sm ) const
		{
			const StableSparseMatrix* p = dynamic_cast<const StableSparseMatrix*>(_sm);

			if( p == NULL )
				throw exception("second parameter mush pass the SparseMatrix type!!");

			Taucs::WriteMatrixIjv( _out, p->GetMatrix() );
		}

		void SerializeStableSparseMatrixFull::Write( ostream& _out, const Matrix* _m ) const
		{
			const StableSparseMatrix* p = dynamic_cast<const StableSparseMatrix*>(_m);

			if( p == NULL )
				throw exception("second parameter mush pass the StableSparseMatrix type!!");

			unsigned int m = p->GetNumRows(), n = p->GetNumCols();

			if( p->IsSymmetric() )
			{
				int flags = p->Flags();

				if( flags & TAUCS_LOWER )
					_write_symm_taucs_lower_( _out, p );
				else if ( flags & TAUCS_UPPER )
					_write_symm_taucs_upper_( _out, p );
			}
			else
			{
				for( unsigned int i = 0 ; i < m ; ++i )
				{
					for( unsigned int j = 0 ; j < n ; ++j )
						_out << p->GetElement( i, j ) << "  ";
					_out << endl;
				}
			}
		}

		void SerializeStableSparseMatrixFull::_write_symm_taucs_lower_( ostream& _out, const StableSparseMatrix* _sm ) const
		{
			assert( _sm && _sm->IsSymmetric() && _sm->Flags() & TAUCS_LOWER );

			unsigned int m = _sm->GetNumRows(), n = _sm->GetNumCols();

			for( unsigned int i = 0 ; i < m ; ++i )
			{
				for( unsigned int j = 0 ; j < n ; ++j )
				{
					if( j <= i )
						_out << _sm->GetElement( i, j ) << "  ";
					else
						_out << _sm->GetElement( j, i ) << "  ";
				}
				_out << endl;
			}
		}

		void SerializeStableSparseMatrixFull::_write_symm_taucs_upper_( ostream& _out, const StableSparseMatrix* _sm ) const
		{
			assert( _sm && _sm->IsSymmetric() && _sm->Flags() & TAUCS_UPPER );

			unsigned int m = _sm->GetNumRows(), n = _sm->GetNumCols();

			for( unsigned int i = 0 ; i < m ; ++i )
			{
				for( unsigned int j = 0 ; j < n ; ++j )
				{
					if( i <= j )
						_out << _sm->GetElement( i, j ) << "  ";
					else
						_out << _sm->GetElement( j, i ) << "  ";
				}
				_out << endl;
			}		
		}

	}

	template<> SerializeStableSparseMatrixIjv*		Singleton<SerializeStableSparseMatrixIjv>::ms_Singleton		= 0;
	template<> SerializeStableSparseMatrixFull*		Singleton<SerializeStableSparseMatrixFull>::ms_Singleton	= 0;

	static SerializeStableSparseMatrixIjv	gs_SerializeStableSparseMatrixIjv;
	static SerializeStableSparseMatrixFull	gs_SerializeStableSparseMatrixFull;

	/*
	Note:
	StableSparseMatrix class 裡的 m_pTaucsMatrix :
	指向由 taucs_ccs_create 所做出來的物件，而不是自己 new 出來了，
	等於是用 StableSparseMatrix 再包一層而己.
	*/

	StableSparseMatrix::StableSparseMatrix()
		: SparseMatrix()
	{
		m_pTaucsMatrix = NULL;
	}

	StableSparseMatrix::StableSparseMatrix( const vector< map<unsigned int,double>* >& cols, int nRows, int flags )
		: SparseMatrix()
	{
		m_pTaucsMatrix = NULL;
		this->Create( cols, nRows, flags );
	}

	StableSparseMatrix::StableSparseMatrix( const StableSparseMatrix& _tsm )
		: SparseMatrix()
	{
		m_pTaucsMatrix = NULL;
		*this = _tsm;
	}

	StableSparseMatrix::StableSparseMatrix( const GeneralSparseMatrix& _sm )
		: SparseMatrix(_sm)
	{
		m_pTaucsMatrix = NULL;
		*this = _sm;	
	}

	StableSparseMatrix::~StableSparseMatrix()
	{
		Free();
	}

	StableSparseMatrix::StableSparseMatrix( const taucs_ccs_matrix* _tcm )
		: SparseMatrix()
	{
		m_pTaucsMatrix = NULL;
		m_pTaucsMatrix = Taucs::CloneMatrix(_tcm);
	}

	StableSparseMatrix& StableSparseMatrix::operator = ( const StableSparseMatrix& _tsm )
	{
		if( this != &_tsm )
		{
			*static_cast<SparseMatrix*>(this) = *static_cast<const SparseMatrix*>(&_tsm);

			// free its data 
			Free();

			// copy from _tsm object.
			m_pTaucsMatrix = Taucs::CloneMatrix( _tsm.m_pTaucsMatrix );
		}

		return *this;
	}

	StableSparseMatrix& StableSparseMatrix::operator = ( const GeneralSparseMatrix& _sm )
	{
		Free();

		if( _sm.IsSymmetric() )
		{
			// 建立它的下半角資料.
			typedef map<unsigned int, double>::const_iterator citerator;

			vector< map<unsigned int, double>* > lower( _sm.GetNumCols() );
			for( unsigned int i = 0 ; i < lower.size() ; ++i )
				lower[i] = new map<unsigned int, double>();

			const vector< map<unsigned int, double>* >& full = _sm.Elements();
			assert( _sm.GetNumCols() == full.size() );

			for( unsigned int col = 0 ; col < _sm.GetNumCols() ; ++col )
				lower[col]->insert( full[col]->lower_bound(col), full[col]->end() );

			if( !Create( lower, (int)full.size(), TAUCS_DOUBLE | TAUCS_LOWER | TAUCS_SYMMETRIC ) )
				throw exception( "operator = GeneralSparseMatrix Failed!" );	

			for( unsigned int i = 0 ; i < lower.size() ; ++i )
				delete lower[i];
			lower.clear();
		}
		else
		{
			if( !Create( _sm.Elements(), _sm.GetNumRows(), TAUCS_DOUBLE ) )
				throw exception( "operator = GeneralSparseMatrix Failed!" );		
		}

		return *this;
	}

	Matrix* StableSparseMatrix::Clone() const
	{
		assert( m_pTaucsMatrix );
		return new StableSparseMatrix(*this);
	}

	/// reset the matrix's data.
	void StableSparseMatrix::Free()
	{
		if( m_pTaucsMatrix )
		{
			// 利用 taucs 提供的 function 作 delete.
			Taucs::Free( m_pTaucsMatrix );
			m_pTaucsMatrix = 0;
		}
	}

	void StableSparseMatrix::Set( taucs_ccs_matrix* pMat )
	{
		if( m_pTaucsMatrix )
			Taucs::Free( m_pTaucsMatrix );
		m_pTaucsMatrix = pMat;
	}

	bool StableSparseMatrix::IsSymmetric() const
	{
		assert( m_pTaucsMatrix );
		return (m_pTaucsMatrix->flags & TAUCS_SYMMETRIC) ? true : false;
	}

	/// transpose matrix.
	Matrix* StableSparseMatrix::Transpose() const
	{
		// the information about this object.
		assert( m_pTaucsMatrix );
		int rows = GetNumRows();
		int cols = GetNumCols();
		int nnz = GetNumNonZero();
		int flags = m_pTaucsMatrix->flags;
		const int* colptr = m_pTaucsMatrix->colptr;
		const int* rowind = m_pTaucsMatrix->rowind;

		// create new matrix to store the transpose matrix.
		StableSparseMatrix* result = new StableSparseMatrix();

		// create transpose's size matrix.
		result->m_pTaucsMatrix = Taucs::Create( cols, rows, nnz, flags );

		if( result->m_pTaucsMatrix == NULL )
			throw exception( "call Taucs::Create failed - in StableSparseMatrix::Transpose" );

		if( IsSymmetric() ) 
		{
			// symmetric - just copy the matrix
			memcpy( result->m_pTaucsMatrix->colptr, colptr, sizeof(int) * (cols + 1) );
			memcpy( result->m_pTaucsMatrix->rowind, rowind, sizeof(int) * nnz );
			memcpy( result->m_pTaucsMatrix->values.d, m_pTaucsMatrix->values.d, sizeof(double) * (nnz));
		}
		else
		{
			// non-symmetric matrix -> need to build data structure.
			// we'll go over the columns and build the rows

			vector< vector<int> >       compress_rows( rows );
			vector< vector<double> >	trans_values( rows );

			// go over the columns.
			for( int c = 0 ; c < cols ; ++c ) 
			{
				// iterative times : c 行的非 0 數目.
				for ( int r = colptr[c] ; r < colptr[c+1] ; ++r ) 
				{
					compress_rows[ rowind[r] ].push_back(c);
					trans_values[ rowind[r] ].push_back( m_pTaucsMatrix->values.d[r] );
				}
			}

			// copying the rows as columns in result matrix
			int cind = 0;
			for( int r = 0; r < rows ; ++r ) 
			{
				result->m_pTaucsMatrix->colptr[r] = cind;

				for( unsigned int j = 0 ; j < compress_rows[r].size() ; ++j ) 
				{
					result->m_pTaucsMatrix->rowind[cind] = compress_rows[r][j];
					result->m_pTaucsMatrix->values.d[cind] = trans_values[r][j];
					cind++;
				}
			}

			result->m_pTaucsMatrix->colptr[rows] = cind;	
		}

		return result;
	}

	double StableSparseMatrix::GetElement( unsigned int _row_index, unsigned int _col_index ) const
	{
		assert( m_pTaucsMatrix );	
		assert( 0u <= _row_index && _row_index < GetNumRows() && 
			    0u <= _col_index && _col_index < GetNumCols() );

		if( m_pTaucsMatrix->flags & TAUCS_SYMMETRIC )
		{
			if( m_pTaucsMatrix->flags & TAUCS_LOWER )
			{	
				if( _col_index > _row_index )
					std::swap( _row_index, _col_index );
			}
			else if( m_pTaucsMatrix->flags & TAUCS_UPPER )
			{
				if( _row_index > _col_index )
					std::swap( _row_index, _col_index );
			}
		}		

		// here i call the (i, j) is the ( row_index, col_index ).
		// 找 A(i, j) 相當於是先找到第 j 行的第一個非 0 項，然後再從此行裡找到第 i 列的值.
		// 不過有可能在 rowind 裡不存在第 i 列，代表該值為 0.

		// step1 : 取得第 j 行第一個非 0 項的 index. 
		unsigned int nnz_index_of_col_j = m_pTaucsMatrix->colptr[ _col_index ];

		// step2 : 計算此第 j 行中非 0 項的個數.
		unsigned int nnz_of_col_j = m_pTaucsMatrix->colptr[ _col_index+1 ] - m_pTaucsMatrix->colptr[ _col_index ];

		// step3 : 從該第 j 行裡找第 i 列.
		for( unsigned int k = 0 ; k < nnz_of_col_j ; ++k )
		{
			if( m_pTaucsMatrix->rowind[ nnz_index_of_col_j ] == (int)_row_index )
				return m_pTaucsMatrix->values.d[ nnz_index_of_col_j ];
			nnz_index_of_col_j++;
		}

		// can't find the (i, j)
		return 0.0;
	}

	unsigned int StableSparseMatrix::GetNumNonZero() const 
	{
		assert( m_pTaucsMatrix );
		return m_pTaucsMatrix->colptr[ GetNumCols() ];
	}

	unsigned int StableSparseMatrix::GetNumRows() const
	{
		assert( m_pTaucsMatrix );
		return m_pTaucsMatrix->m;
	}

	unsigned int StableSparseMatrix::GetNumCols() const
	{
		assert( m_pTaucsMatrix );
		return m_pTaucsMatrix->n;
	}

	// Multiplies this matrix by x and stores the result in b. Assumes all memory has been allocated
	// and the sizes match; assumes this matrix is not symmetric!!
	void StableSparseMatrix::MultiplyVector( double* _x, double* _b ) const
	{
		assert( m_pTaucsMatrix );

		// make b all zero
		memset( _b, 0, GetNumRows() * sizeof(double) );

		for( unsigned int col = 0 ; col < GetNumCols() ; ++col ) 
		{
			// going over column col of matA, multiplying
			// it by x[col] and setting the appropriate values
			// of vector b
			for( int p = m_pTaucsMatrix->colptr[col] ; p < m_pTaucsMatrix->colptr[col+1] ; ++p ) 
				_b[ m_pTaucsMatrix->rowind[p] ] += _x[col] * m_pTaucsMatrix->values.d[p];
		}
	}

	void StableSparseMatrix::MultiplyVector( const vector<double>& _x, double* _b ) const
	{		
		assert( m_pTaucsMatrix );
		
		// make b all zero
		memset( _b, 0, GetNumRows() * sizeof(double) );

		for( unsigned int col = 0 ; col < GetNumCols() ; ++col ) 
		{
			// going over column col of matA, multiplying
			// it by x[col] and setting the appropriate values
			// of vector b
			for( int p = m_pTaucsMatrix->colptr[col] ; p < m_pTaucsMatrix->colptr[col+1] ; ++p ) 
				_b[ m_pTaucsMatrix->rowind[p] ] += _x[col] * m_pTaucsMatrix->values.d[p];
		}
	}
	
	StableSparseMatrix* StableSparseMatrix::MultiplyNonSymMatrix( const StableSparseMatrix& _tsm ) const
	{	
		assert( m_pTaucsMatrix );

		const taucs_ccs_matrix* matA = this->m_pTaucsMatrix;
		const taucs_ccs_matrix* matB = _tsm.m_pTaucsMatrix;

		// Compatibility of dimensions        
		if( matA->m != matB->n )
			return NULL;

		if( (matA->flags & TAUCS_SYMMETRIC) || (matB->flags & TAUCS_LOWER) )
			return NULL;

		// (m x n)*(n x k) = (m x k)
		int m = matA->m;
//		int n = matA->n;
		int k = matB->n;

		double biv, valA;
		int rowInd, rowA;
		vector< map<unsigned int, double>* > rowsC(k);

		for( unsigned int i = 0 ; i < rowsC.size() ; ++i )
			rowsC[i] = new map<unsigned int, double>();

		for( int i = 0 ; i < k ; ++i ) 
		{
			// creating column i of C
			map<unsigned int, double>& mapRow2Val = *rowsC[i];

			// travel on bi
			for (int rowptrBi = matB->colptr[i] ; rowptrBi < matB->colptr[i+1] ; ++rowptrBi ) 
			{
				rowInd = matB->rowind[rowptrBi];
				biv = matB->values.d[rowptrBi];

				// make biv*a_{rowInd} and insert into mapRow2Val
				for (int rowptrA=matA->colptr[rowInd];rowptrA<matA->colptr[rowInd+1];++rowptrA)
				{
					rowA = matA->rowind[rowptrA];
					valA = matA->values.d[rowptrA];

					// insert valA*biv into map
					map<unsigned int, double>::iterator it = mapRow2Val.find(rowA);

					if( it == mapRow2Val.end() ) // first time
						mapRow2Val[rowA] = valA*biv;
					else 
						it->second = it->second + valA*biv;
				}
			}
			// now column i is created
		}

		StableSparseMatrix* result = new StableSparseMatrix( rowsC, m, TAUCS_DOUBLE );

		for( unsigned int i = 0 ; i < rowsC.size() ; ++i )
			delete rowsC[i];
		rowsC.clear();

		return result;
	}

	// for usage when it's known that the result is symmetric,
	// like A^T * A
	StableSparseMatrix* StableSparseMatrix::MultiplyNonSymmMatSymmResult( const StableSparseMatrix& _tsm ) const
	{
		assert( m_pTaucsMatrix );

		const taucs_ccs_matrix* matA = this->m_pTaucsMatrix;
		const taucs_ccs_matrix* matB = _tsm.m_pTaucsMatrix;

		// Compatibility of dimensions  
		if( this->GetNumRows() != _tsm.GetNumCols() || this->GetNumCols() != _tsm.GetNumRows() )
			return NULL;

		if( m_pTaucsMatrix->flags & TAUCS_SYMMETRIC  || _tsm.m_pTaucsMatrix->flags & TAUCS_LOWER )
			return NULL;

		// (m x n)*(n x m) = (m x m)
		int m = this->GetNumRows();
//		int n = this->GetNumCols();

		double biv, valA;
		int rowInd, rowA;

		vector< map<unsigned int, double>* > rowsC(m);
		for( unsigned int i = 0 ; i < rowsC.size() ; ++i )
			rowsC[i] = new map<unsigned int, double>();

		for( int i = 0 ; i < m ; ++i ) 
		{
			// creating column i of C
			map<unsigned int, double>& mapRow2Val = *rowsC[i];

			// travel on bi
			for( int rowptrBi = matB->colptr[i] ; rowptrBi < matB->colptr[i+1] ; ++rowptrBi ) 
			{
				rowInd = matB->rowind[rowptrBi];
				biv = matB->values.d[rowptrBi];

				// make biv*a_{rowInd} and insert into mapRow2Val
				// Ignore anything above the diagonal!!
				for( int rowptrA = matA->colptr[rowInd] ; rowptrA < matA->colptr[rowInd+1] ; ++rowptrA ) 
				{
					rowA = matA->rowind[rowptrA];

					if( rowA >= i ) 
					{
						valA = matA->values.d[rowptrA];

						// insert valA*biv into map
						map<unsigned int, double>::iterator it = mapRow2Val.find(rowA);

						if(it == mapRow2Val.end()) // first time
							mapRow2Val[rowA] = valA*biv;
						else 
							it->second = it->second + valA*biv;
					}
				}
			}
			// now column i is created
		}

		StableSparseMatrix* result = new StableSparseMatrix( rowsC, m, TAUCS_DOUBLE | TAUCS_SYMMETRIC | TAUCS_LOWER );

		for( unsigned int i = 0 ; i < rowsC.size() ; ++i )
			delete rowsC[i];
		rowsC.clear();

		return result;
	}

	bool StableSparseMatrix::Create( const vector< map<unsigned int,double>* >& cols, int nRows, int flags )
	{
		Free();

		// count nnz:
		int nCols = static_cast<int>( cols.size() );

		int nnz = 0;
		for (int counter=0; counter < nCols; ++counter)
			nnz += static_cast<int>(cols[counter]->size());

		m_pTaucsMatrix = Taucs::Create( nRows, nCols, nnz, flags );

		if( m_pTaucsMatrix == NULL )
			return false;

		// copy cols into matC
		map<unsigned int,double>::const_iterator rit;

		int rowptrC = 0;

		for (int c=0;c<nCols;++c) 
		{
			m_pTaucsMatrix->colptr[c] = rowptrC;

			for (rit = cols[c]->begin();rit!= cols[c]->end();++rit) 
			{
				m_pTaucsMatrix->rowind[rowptrC]=rit->first;
//				int ind = rit->first;
				m_pTaucsMatrix->values.d[rowptrC]=rit->second;
//				double val = rit->second;
				++rowptrC;
			}
		}
		m_pTaucsMatrix->colptr[nCols]=nnz;

		return true;
	}

	SparseMatrix* StableSparseMatrix::NormalEquationForm() const
	{
		assert( m_pTaucsMatrix );
		StableSparseMatrix* transpose = (StableSparseMatrix*) Transpose();

		if( this->IsSymmetric() )
			throw exception("StableSparseMatrix::NormalEquationForm : this object can't be a symmetric matrix" );

		StableSparseMatrix* result = transpose->MultiplyNonSymmMatSymmResult(*this);
		delete transpose;
		return result;
	}

	int StableSparseMatrix::Flags() const
	{
		assert( m_pTaucsMatrix );
		return m_pTaucsMatrix->flags;
	}

	void StableSparseMatrix::Write( ostream& _out, SerializeMatrix* _format ) const
	{
		_format->Write( _out, this );
	}

	SerializeMatrix* StableSparseMatrix::GetSerializeIjv() const
	{
		return Singleton<SerializeStableSparseMatrixIjv>::GetInstancePtr();
	}

	SerializeMatrix* StableSparseMatrix::GetSerializeFull() const
	{
		return Singleton<SerializeStableSparseMatrixFull>::GetInstancePtr();
	}

	taucs_ccs_matrix* StableSparseMatrix::GetMatrix()
	{
		return m_pTaucsMatrix;
	}

	const taucs_ccs_matrix* StableSparseMatrix::GetMatrix() const
	{
		return m_pTaucsMatrix;
	}

	/// 做 LLT factorize.
	int StableSparseMatrix::LLTFactorize( void*& out_l ) const
	{
		return Taucs::LinSolveLLTFactor( m_pTaucsMatrix, out_l );
	}
}