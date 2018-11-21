#ifndef _LINEARSYSTEMLIB_HARD_CONSTRAINT_SLS_BUILDER_H_
#define _LINEARSYSTEMLIB_HARD_CONSTRAINT_SLS_BUILDER_H_

#include "linearsystembuilder.h"
#include "HardConstraintSLS.h"
#include <cassert>

namespace LinearSystemLib
{
	/// 支援 hard constraints 的 sparse linear system builder.
	/// 用法 :
	///		繼承此 class 後實作 doSetupConstraintConditions and doSetupEquation。
	///     建立 child object 後再呼叫 BuildLinearSystem.
	/// ConType : A*x = B 的 B 的 type, 此 ConType 可有多個維度,將由 Extractor 來負責
	///		      取出第 i 維的資料.
	/// note : 1. 建造出來的 linear system 為 HardConstraintSparseLinearSystem<ConType>
	///		   2. 建造出來的 linear system 維度依然為 n, 有含 constraint 的部份.
	template < class ConType, class Extractor >
	class HardConstraintSLSBuilder : public LinearSystemBuilder
	{
	public:

		/// constructor
		/// n : 所有變數的個數，“包含未知解個數+己知解個數”
		/// dim : 解的維度，若在 3D 上則 dim = 3. etc.
		HardConstraintSLSBuilder( unsigned int n, unsigned int dim )
		: LinearSystemBuilder( n, dim )
		{
			linearSystem() = new HardConstraintSLS<ConType>( dim );
			mc_Extractor = Extractor();
		}

		// this can be a template method.
		// if you use this class, don't override this function.
		virtual void BuildLinearSystem()
		{
			// 先設置好所有的 constraint conditions.
			doSetupConstraintConditions();

			HardConstraintSLS<ConType>* linear_system = 
				(HardConstraintSLS<ConType>*) linearSystem();

			// 含有各個 constraint condition 的資料
			const ConstraintContainer<ConType>& cons_data = linear_system->Constraints();

			const unsigned int dim = dimension();						// the dimension.
			const unsigned int nTotalCons = cons_data.size();			// constraint 的個數
			const unsigned int nTotalVars = numVariables();				// total variable, 含己知點.
			unsigned int i;

			//--------------------------------------------------------------------------
			// 下列變數為 Matrix A * x = B 的 A 和 B.
			//--------------------------------------------------------------------------

			GeneralSparseMatrix A( nTotalVars, nTotalVars );

			double** B = new double*[dim];
			for( i = 0 ; i < dim ; ++i )
			{
				B[i] = new double[ nTotalVars ];
				memset( &B[i][0], 0, sizeof( double ) * nTotalVars );
			}

			//--------------------------------------------------------------------------
			// 下列變數用在存某 equation 的 var indices, 該 element 在 matrix 的值
			// 以及該 equation 的值, ex: 4 * v2 + 6 * v5 + 10 * v10 = 10
			//--------------------------------------------------------------------------

			vector< unsigned int >	equ_col_index;
			vector< double >		equ_col_value;
			double* equ_b_value = new double[dim];

			equ_col_index.reserve(8);
			equ_col_value.reserve(8);

			// 還須不須要繼續判斷是否為 constraint vertex index, 主要用來加速用.
			bool dont_judge_is_cons_index_flag = false;
			const size_t size = sizeof(double) * dim;

			ConstraintContainer<ConType>::const_iterator citer = cons_data.begin();

			for( unsigned int row_index = 0u ; row_index < nTotalVars ; ++row_index )
			{
				// 分別考慮 variable index 為 constraint vertex or not 的 case.
				if( dont_judge_is_cons_index_flag || row_index != cons_data.Index(citer) )
				{	
					// 表示此 variable index 不為 constraint vertex 的 index.
					// 或是不再須要判斷是否為 constraint vertex index 了. <= 加速用.

					// erase preious's equation data.
					equ_col_index.erase( equ_col_index.begin(), equ_col_index.end() );
					equ_col_value.erase( equ_col_value.begin(), equ_col_value.end() );
					memset( equ_b_value, 0, size );

					// 取得第 row_index 個 equation (它會被設定在第 row_index 列) 
					doSetupEquation( row_index, equ_col_index, equ_col_value, equ_b_value );

					// set up A and B 的值.
					assign_equation_to_A_and_B( A, B, row_index, equ_col_index, equ_col_value, equ_b_value, cons_data );
				}
				else 
				{
					// 此 variable index 為 constraint variable.
					// 在此 case 中, 此列將改填 vi = b <= constraint condition.

					A.SetElement( row_index, row_index, 1.0 );

					// set up B 的值.
					const ConType& value = *cons_data.Value( citer );
					for( i = 0u ; i < dim ; ++i )
						B[i][row_index] = double( mc_Extractor( value, i ) );

					if( ++citer == cons_data.end() )
						dont_judge_is_cons_index_flag = true;
				}
			}

			delete[] equ_b_value;

			linear_system->GetA() = new StableSparseMatrix( A );
			linear_system->GetB() = B;
		}

	protected: // function that children class must implement.

		/// 設置好所有的 constraint conditions.
		/// note : 在設定 constraint 時，利用 setConstaints or setConstaint method 設定
		virtual void doSetupConstraintConditions() = 0;

		/// 請傳給我 Matrix A 某列的非 0 位置和它的值，及該列對應到的 B 的值.
		virtual void doSetupEquation( unsigned int row_index, 
			vector<unsigned int>& col_indices, vector<double>& coefs, double*& bvalue ) = 0;

	protected: // 給 children class 設定 constraint 用 和插入係數 的輔助函式.

		/// 代表線性系統的所有 constraint condition 一次設定好.
		void setConstaints( const ConstraintContainer<ConType>& cons )
		{
			((HardConstraintSLS<ConType>*) linearSystem())->setConstraints( cons );
		}

		/// 可以一個一個設定 constraint condition.
		void setConstraint( unsigned int index, const ConType* value )
		{
			assert( value != NULL );
			((HardConstraintSLS<ConType>*) linearSystem())->setConstraint( index, value );
		}


		/// 插入係數的輔助函式, 因為 col_indices 必須為 sorted 的順序,
		/// 因此提供此 function 來讓 col_index 插入後, col_indices 仍得為 sorted.
		/// col_index ：該 coefficient 所在的 column index
		/// value : coefficient value.
		void insertElement( 
			vector<unsigned int>& col_indices, vector<double>& coefs, 
			unsigned int col_index, double value )
		{
			// 先找出應該要插在哪個位置.
			vector<unsigned int>::iterator it1 = 
				upper_bound( col_indices.begin(), col_indices.end(), col_index );

			vector<double>::iterator it2 = coefs.begin();
			advance( it2, distance( col_indices.begin(), it1 ) );

			// 插入 column index and coefficient
			col_indices.insert( it1, col_index );
			coefs.insert( it2, value );
		}

	private: // helper function.

		// 將此 equation assgin 到 A, B 裡面。
		void assign_equation_to_A_and_B( 
			GeneralSparseMatrix& A, double** B, unsigned int row_index,
			const vector<unsigned int>& equ_col_index, const vector<double>& equ_col_value, 
			const double* equ_b_value, const ConstraintContainer<ConType>& cons_data )
		{
			// 紀錄目前已經經過幾個 constraint 了.
			ConstraintContainer<ConType>::const_iterator cit = cons_data.begin();

			unsigned int n = equ_col_index.size();
			unsigned int dim = dimension();							// the dimension.

			unsigned int i = 0u;

			// set up B 的值.
			for( i = 0u ; i < dim ; ++i )
				B[i][ row_index ] = equ_b_value[i];

			i = 0u;

			while( i < n && cit != cons_data.end() )
			{
				if( equ_col_index[i] < cons_data.Index(cit) )
				{
					// 該 col index 不為 constraint vertex index 就直接填
					A.SetElement( row_index, equ_col_index[i], equ_col_value[i] );
					++i;
				}
				else
				{
					// 若該 col index 為 constraint vertex index 則不填入 matrix A
					// 且必須修改 right-hand side 的 B.
					if( equ_col_index[i] == cons_data.Index(cit) )
					{
						// update b's value.
						const ConType& value = *(cons_data.Value(cit));
						for( unsigned int j = 0u ; j < dim ; ++j )
							B[j][ row_index ] -= equ_col_value[i] * mc_Extractor( value, j );
						++i;
					}

					++cit;
				}
			}

			for( ; i < n ; ++i )
				A.SetElement( row_index, equ_col_index[i], equ_col_value[i] );
		}

	protected:
		
		Extractor mc_Extractor;
	};

	//===============================================================================================
}

#endif
