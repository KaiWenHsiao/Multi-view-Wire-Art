#include "GeneralSparseMatrix.h"

#include "Singleton.h"

#include <algorithm>
#include <cassert>
#include <cfloat>

using namespace std;

namespace LinearSystemLib
{
	namespace 
	{
		inline bool IsZero( double a )
		{
			return ( a < 0 ? (-a < DBL_EPSILON) : (a < DBL_EPSILON) );
		}

		class SerializeGeneralSparseMatrixFull 
			: public SerializeMatrix
			, public Singleton<SerializeGeneralSparseMatrixFull>
		{
		public:
			// �ĤG�ӥ����O�i�H�૬�� GeneralSparseMatrix�A�]���O for GeneralSparseMatrix
			virtual void Write( ostream& _out, const Matrix* _sm ) const;
		};

		class SerializeGeneralSparseMatrixIjv 
			: public SerializeMatrix
			, public Singleton<SerializeGeneralSparseMatrixIjv>
		{
		public:
			// �ĤG�ӥ����O�i�H�૬�� GeneralSparseMatrix�A�]���O for GeneralSparseMatrix
			virtual void Write( ostream& _out, const Matrix* _sm ) const;
		};

		//-----------------------------------------------------------------------

		void SerializeGeneralSparseMatrixIjv::Write( ostream& _out, const Matrix* _m ) const
		{
			const GeneralSparseMatrix* p = dynamic_cast<const GeneralSparseMatrix*>(_m);

			if( p == NULL )
				throw exception("second parameter mush pass the GeneralSparseMatrix type!!");

			const vector< map<unsigned int, double>* >& elements = p->Elements();
			typedef map<unsigned int, double>::const_iterator citerator;

			if( p->IsSymmetric() )
			{
				for( unsigned int col = 0 ; col < p->GetNumCols() ; ++col )
				{
					citerator cit = elements[col]->lower_bound( col );
					for( ; cit != elements[col]->end() ; ++cit )
					{
						_out << (cit->first+1) << " " << (col+1) << " " << cit->second << endl;
						if( cit->first != col )  // �D�﨤�u case.
							_out << (col+1) << " " << (cit->first+1) << " " << cit->second << endl;
					}		
				}
			}
			else
			{
				for( unsigned int col = 0 ; col < p->GetNumCols() ; ++col )
					for( citerator cit = elements[col]->begin() ; cit != elements[col]->end() ; ++cit )
						_out << (cit->first+1) << " " << (col+1) << " " << cit->second << endl;
			}
		}

		void SerializeGeneralSparseMatrixFull::Write( ostream& _out, const Matrix* _m ) const
		{
			const GeneralSparseMatrix* p = dynamic_cast<const GeneralSparseMatrix*>(_m);

			if( p == NULL )
				throw exception("second parameter mush pass the GeneralSparseMatrix type!!");

			for( unsigned int i = 0 ; i < p->GetNumRows() ; ++i )
			{
				for( unsigned int j = 0 ; j < p->GetNumCols() ; ++j )
					_out << p->GetElement( i, j ) << " ";
				_out << endl;
			}
		}
	}

	template<> SerializeGeneralSparseMatrixIjv*		Singleton<SerializeGeneralSparseMatrixIjv>::ms_Singleton	= 0;
	template<> SerializeGeneralSparseMatrixFull*	Singleton<SerializeGeneralSparseMatrixFull>::ms_Singleton	= 0;

	static SerializeGeneralSparseMatrixIjv	gs_SerializeGeneralSparseMatrixIjv;
	static SerializeGeneralSparseMatrixFull gs_SerializeGeneralSparseMatrixFull;



	GeneralSparseMatrix::GeneralSparseMatrix( unsigned int _m, unsigned int _n )
		: SparseMatrix()
	{
		m_uRows = _m;
		
		m_cElements.resize( _n, 0 );
		for( unsigned int i = 0 ; i < _n ; ++i )
			m_cElements[i] = new map<unsigned int, double>(); 
	}

	GeneralSparseMatrix& GeneralSparseMatrix::operator = ( const GeneralSparseMatrix& _sm )
	{
		if( this != &_sm )
		{
			*static_cast<SparseMatrix*>(this) = *static_cast<const SparseMatrix*>(&_sm);

			Free();

			m_uRows = _sm.m_uRows;

			m_cElements.resize( _sm.m_cElements.size() );

			for( unsigned int i = 0 ; i < _sm.m_cElements.size() ; ++i )
				m_cElements[i] = new map<unsigned int, double>( *_sm.m_cElements[i] );
		}

		return *this;
	}

	void GeneralSparseMatrix::Free()
	{
		m_uRows = 0u;
		for( unsigned int i = 0 ; i < m_cElements.size() ; ++i )
		{
			delete m_cElements[i]; 
			m_cElements[i] = 0;
		}

		m_cElements.erase( m_cElements.begin(), m_cElements.end() );
	}

	void GeneralSparseMatrix::SetElement( unsigned int _row_index, unsigned int _col_index, double _value ) 
	{
		assert( 0 <= _row_index && _row_index < m_uRows && 0 <= _col_index && _col_index < GetNumCols() );

		map<unsigned int, double>::iterator it = m_cElements[_col_index]->find(_row_index);
		map<unsigned int, double>::iterator end = m_cElements[_col_index]->end();

		if( it != end )
		{	
			// �Y _value �O zero, �N��Ʊ�N�쥻���Ȫ��a��M�� 0.
			// �Y�_�A�h�N��Q�� _value ���N�쥻����.

			if( IsZero(_value) )
				m_cElements[_col_index]->erase( _row_index );
			else
				it->second = _value;
		}
		else // �쥻�b (row_index, col_index) ���a�謰 0
		{
			// �Y _value ���� 0 �h���J�A�� 0 �h��������ʧ@�C
			if( !IsZero(_value) )
				(*m_cElements[_col_index])[_row_index] = _value;	
		}
	}



	Matrix* GeneralSparseMatrix::Transpose() const
	{
		GeneralSparseMatrix* result = new GeneralSparseMatrix( GetNumCols(), GetNumRows() );
		vector< map<unsigned int, double>* >& matrix = result->m_cElements;

		unsigned int col_index = 0u;

		for( vector< map<unsigned int, double>* >::const_iterator out_cit = m_cElements.begin() ; 
			out_cit != m_cElements.end() ; ++out_cit )
		{
			for( map<unsigned int, double>::const_iterator in_cit = (*out_cit)->begin() ; 
				in_cit != (*out_cit)->end() ; ++in_cit )
			{
				unsigned int row_index = in_cit->first; // the key is the row index.
				double value = in_cit->second;
				(*matrix[row_index])[col_index] = value;
			}

			++col_index;
		}

		return static_cast<Matrix*>(result);
	}

	void GeneralSparseMatrix::Create( unsigned int _rows, unsigned int _cols )
	{
		assert( _rows > 0 && _cols > 0 );

		Free();

		m_uRows = _rows;

		m_cElements.resize( _cols, 0 );
		for( unsigned int i = 0 ; i < _cols ; ++i )
			m_cElements[i] = new map<unsigned int, double>();
	}

	bool GeneralSparseMatrix::IsSymmetric() const
	{
		if( m_cElements.size() != this->m_uRows )
			return false;

		// �Q�ΤW�b�����T���Ψӧ�U�b�����T����.
		typedef map<unsigned int, double>::const_iterator citerator;

		for( unsigned int col = 0 ; col < this->GetNumCols() ; ++col )
		{
			citerator end = m_cElements[col]->lower_bound(col);
			for( citerator it = m_cElements[col]->begin() ; it != end ; ++it )
			{
				unsigned int row = it->first;
				citerator sym_it = m_cElements[row]->find(col);
				if( sym_it == m_cElements[row]->end() || it->second != sym_it->second )
					return false;
			}
		}

		return true;
	}

	void GeneralSparseMatrix::DeleteRows( unsigned int _first, unsigned int _last )
	{
		assert( 0 <= _first && _first < GetNumRows() && 0 <= _last && 
			_last <= GetNumRows() && _last > _first );

		const unsigned int num_delete = _last - _first;

		typedef map<unsigned int, double>::iterator		iterator;
		typedef map<unsigned int, double>::value_type	value_type;

		// store all the rows and it's value whose row index is greater than or equal to "last".
		// the reason why "equal to" is : front close and back open.
		vector< pair<unsigned int, double> > tmp; 
		tmp.reserve( m_uRows );

		for( int i = 0 ; i < static_cast<int>(m_cElements.size()) ; ++i )
		{
			map<unsigned int, double>& row_indices = *m_cElements[i];

			tmp.erase( tmp.begin(), tmp.end() );

			// step1 : �����X row index >= last �������̨üȦs�_��, �s���ɭ� index �n����n.
			for( iterator lower = row_indices.lower_bound( _last ) ; lower != row_indices.end() ; ++lower )
				tmp.push_back( make_pair(lower->first - num_delete, lower->second) );

			// step2 : ���� row index �b _first(�t)���᪺ elements.
			row_indices.erase( row_indices.lower_bound( _first ), row_indices.end() );

			// step3 : �N�ק�L���ȳ��[�^�h.
			row_indices.insert( tmp.begin(), tmp.end() );
		}

		// update the row number.
		m_uRows = m_uRows - num_delete;
	}

	void GeneralSparseMatrix::DeleteCols( unsigned int _first, unsigned int _last )
	{
		assert( 0 <= _first && _first < GetNumCols() && 0 <= _last && 
			_last <= GetNumCols() && _last > _first );

		vector< map<unsigned int, double>* >::iterator begin = m_cElements.begin();
		vector< map<unsigned int, double>* >::iterator end = m_cElements.begin();

		advance( begin, _first );
		advance( end, _last );

		for( unsigned int i = _first ; i < _last ; ++i )
			delete m_cElements[i];

		// ��檺�ܦC���Χ�.
		m_cElements.erase( begin, end );
	}

	void GeneralSparseMatrix::DeleteColsStoreTo( unsigned int _first, unsigned int _last, 
		vector< map<unsigned int,double>* >& _dest )
	{
		assert( 0 <= _first && _first < GetNumCols() && 0 <= _last && 
			_last <= GetNumCols() && _last > _first );

		vector< map<unsigned int, double>* >::iterator begin = m_cElements.begin();
		vector< map<unsigned int, double>* >::iterator end = begin;

		advance( begin, _first );
		advance( end, _last );

		_dest.insert( _dest.end(), begin, end );

		// ��ª� erase, ���n delete memory.
		m_cElements.erase( begin, end );
	}

	GeneralSparseMatrix* GeneralSparseMatrix::Multiply( const GeneralSparseMatrix& _sm ) const
	{
		// Compatibility of dimensions        
		if( GetNumCols() != _sm.GetNumRows() )
			return NULL;

		// (m x n)*(n x k) = (m x k)
		int m = GetNumRows();
		//int n = GetNumCols();
		int k = _sm.GetNumCols();

		GeneralSparseMatrix* result = new GeneralSparseMatrix( m, k );

		typedef map<unsigned int, double>	Map;
		typedef Map::const_iterator			citerator;

		for( int i = 0 ; i < k ; ++i )
		{
			// �ΨӦs A*B = C �� C �� column i ����.
			Map& rows_of_col_i = *(result->m_cElements[i]);

			// �s B �� column i ����.
			const Map& b_col_i = *(_sm.m_cElements[i]);

			// iterative for each row of matrix A.
			for( int j = 0 ; j < m ; ++j )
			{
				double sum = 0.0;

				// travel on column of matrix b.
				for(citerator first = b_col_i.begin() ; first != b_col_i.end() ; ++first )
				{
					unsigned int row = first->first;
					double value = first->second;

					citerator cor_it = m_cElements[row]->find(j);

					if( cor_it != m_cElements[row]->end() )
						sum = sum + value * cor_it->second;
				}

				if( !IsZero(sum) )
					rows_of_col_i[j] = sum;
			}

			// now column i is created
		}

		return result;
	}



	// Multiplies this matrix by x and stores the result in b. Assumes all memory has been allocated
	// and the sizes match assumes this matrix is not symmetric!!
	void GeneralSparseMatrix::MultiplyVector( double* _x, double* _b ) const
	{
		int rows = GetNumRows();
		int cols = GetNumCols();

		memset( _b, 0, rows * sizeof(double) );

		for( int col_index = 0 ; col_index < cols ; ++col_index )
		{
			const map<unsigned int, double>& col = *(m_cElements[col_index]);

			for( map<unsigned int, double>::const_iterator cit = col.begin() ; cit != col.end() ; ++cit )
			{
				unsigned int row_index = cit->first;
				double value = cit->second;

				_b[row_index] += value * _x[col_index];
			}
		}
	}


	SerializeMatrix* GeneralSparseMatrix::GetSerializeIjv() const
	{
		return Singleton<SerializeGeneralSparseMatrixIjv>::GetInstancePtr();
	}

	SerializeMatrix* GeneralSparseMatrix::GetSerializeFull() const
	{
		return Singleton<SerializeGeneralSparseMatrixFull>::GetInstancePtr();
	}	
}