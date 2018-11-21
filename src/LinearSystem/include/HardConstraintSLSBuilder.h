#ifndef _LINEARSYSTEMLIB_HARD_CONSTRAINT_SLS_BUILDER_H_
#define _LINEARSYSTEMLIB_HARD_CONSTRAINT_SLS_BUILDER_H_

#include "linearsystembuilder.h"
#include "HardConstraintSLS.h"
#include <cassert>

namespace LinearSystemLib
{
	/// �䴩 hard constraints �� sparse linear system builder.
	/// �Ϊk :
	///		�~�Ӧ� class ���@ doSetupConstraintConditions and doSetupEquation�C
	///     �إ� child object ��A�I�s BuildLinearSystem.
	/// ConType : A*x = B �� B �� type, �� ConType �i���h�Ӻ���,�N�� Extractor �ӭt�d
	///		      ���X�� i �������.
	/// note : 1. �سy�X�Ӫ� linear system �� HardConstraintSparseLinearSystem<ConType>
	///		   2. �سy�X�Ӫ� linear system ���ר̵M�� n, ���t constraint ������.
	template < class ConType, class Extractor >
	class HardConstraintSLSBuilder : public LinearSystemBuilder
	{
	public:

		/// constructor
		/// n : �Ҧ��ܼƪ��ӼơA���]�t�����ѭӼ�+�v���ѭӼơ�
		/// dim : �Ѫ����סA�Y�b 3D �W�h dim = 3. etc.
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
			// ���]�m�n�Ҧ��� constraint conditions.
			doSetupConstraintConditions();

			HardConstraintSLS<ConType>* linear_system = 
				(HardConstraintSLS<ConType>*) linearSystem();

			// �t���U�� constraint condition �����
			const ConstraintContainer<ConType>& cons_data = linear_system->Constraints();

			const unsigned int dim = dimension();						// the dimension.
			const unsigned int nTotalCons = cons_data.size();			// constraint ���Ӽ�
			const unsigned int nTotalVars = numVariables();				// total variable, �t�v���I.
			unsigned int i;

			//--------------------------------------------------------------------------
			// �U�C�ܼƬ� Matrix A * x = B �� A �M B.
			//--------------------------------------------------------------------------

			GeneralSparseMatrix A( nTotalVars, nTotalVars );

			double** B = new double*[dim];
			for( i = 0 ; i < dim ; ++i )
			{
				B[i] = new double[ nTotalVars ];
				memset( &B[i][0], 0, sizeof( double ) * nTotalVars );
			}

			//--------------------------------------------------------------------------
			// �U�C�ܼƥΦb�s�Y equation �� var indices, �� element �b matrix ����
			// �H�θ� equation ����, ex: 4 * v2 + 6 * v5 + 10 * v10 = 10
			//--------------------------------------------------------------------------

			vector< unsigned int >	equ_col_index;
			vector< double >		equ_col_value;
			double* equ_b_value = new double[dim];

			equ_col_index.reserve(8);
			equ_col_value.reserve(8);

			// �ٶ������n�~��P�_�O�_�� constraint vertex index, �D�n�Ψӥ[�t��.
			bool dont_judge_is_cons_index_flag = false;
			const size_t size = sizeof(double) * dim;

			ConstraintContainer<ConType>::const_iterator citer = cons_data.begin();

			for( unsigned int row_index = 0u ; row_index < nTotalVars ; ++row_index )
			{
				// ���O�Ҽ{ variable index �� constraint vertex or not �� case.
				if( dont_judge_is_cons_index_flag || row_index != cons_data.Index(citer) )
				{	
					// ��ܦ� variable index ���� constraint vertex �� index.
					// �άO���A���n�P�_�O�_�� constraint vertex index �F. <= �[�t��.

					// erase preious's equation data.
					equ_col_index.erase( equ_col_index.begin(), equ_col_index.end() );
					equ_col_value.erase( equ_col_value.begin(), equ_col_value.end() );
					memset( equ_b_value, 0, size );

					// ���o�� row_index �� equation (���|�Q�]�w�b�� row_index �C) 
					doSetupEquation( row_index, equ_col_index, equ_col_value, equ_b_value );

					// set up A and B ����.
					assign_equation_to_A_and_B( A, B, row_index, equ_col_index, equ_col_value, equ_b_value, cons_data );
				}
				else 
				{
					// �� variable index �� constraint variable.
					// �b�� case ��, ���C�N��� vi = b <= constraint condition.

					A.SetElement( row_index, row_index, 1.0 );

					// set up B ����.
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

		/// �]�m�n�Ҧ��� constraint conditions.
		/// note : �b�]�w constraint �ɡA�Q�� setConstaints or setConstaint method �]�w
		virtual void doSetupConstraintConditions() = 0;

		/// �жǵ��� Matrix A �Y�C���D 0 ��m�M�����ȡA�θӦC�����쪺 B ����.
		virtual void doSetupEquation( unsigned int row_index, 
			vector<unsigned int>& col_indices, vector<double>& coefs, double*& bvalue ) = 0;

	protected: // �� children class �]�w constraint �� �M���J�Y�� �����U�禡.

		/// �N��u�ʨt�Ϊ��Ҧ� constraint condition �@���]�w�n.
		void setConstaints( const ConstraintContainer<ConType>& cons )
		{
			((HardConstraintSLS<ConType>*) linearSystem())->setConstraints( cons );
		}

		/// �i�H�@�Ӥ@�ӳ]�w constraint condition.
		void setConstraint( unsigned int index, const ConType* value )
		{
			assert( value != NULL );
			((HardConstraintSLS<ConType>*) linearSystem())->setConstraint( index, value );
		}


		/// ���J�Y�ƪ����U�禡, �]�� col_indices ������ sorted ������,
		/// �]�����Ѧ� function ���� col_index ���J��, col_indices ���o�� sorted.
		/// col_index �G�� coefficient �Ҧb�� column index
		/// value : coefficient value.
		void insertElement( 
			vector<unsigned int>& col_indices, vector<double>& coefs, 
			unsigned int col_index, double value )
		{
			// ����X���ӭn���b���Ӧ�m.
			vector<unsigned int>::iterator it1 = 
				upper_bound( col_indices.begin(), col_indices.end(), col_index );

			vector<double>::iterator it2 = coefs.begin();
			advance( it2, distance( col_indices.begin(), it1 ) );

			// ���J column index and coefficient
			col_indices.insert( it1, col_index );
			coefs.insert( it2, value );
		}

	private: // helper function.

		// �N�� equation assgin �� A, B �̭��C
		void assign_equation_to_A_and_B( 
			GeneralSparseMatrix& A, double** B, unsigned int row_index,
			const vector<unsigned int>& equ_col_index, const vector<double>& equ_col_value, 
			const double* equ_b_value, const ConstraintContainer<ConType>& cons_data )
		{
			// �����ثe�w�g�g�L�X�� constraint �F.
			ConstraintContainer<ConType>::const_iterator cit = cons_data.begin();

			unsigned int n = equ_col_index.size();
			unsigned int dim = dimension();							// the dimension.

			unsigned int i = 0u;

			// set up B ����.
			for( i = 0u ; i < dim ; ++i )
				B[i][ row_index ] = equ_b_value[i];

			i = 0u;

			while( i < n && cit != cons_data.end() )
			{
				if( equ_col_index[i] < cons_data.Index(cit) )
				{
					// �� col index ���� constraint vertex index �N������
					A.SetElement( row_index, equ_col_index[i], equ_col_value[i] );
					++i;
				}
				else
				{
					// �Y�� col index �� constraint vertex index �h����J matrix A
					// �B�����ק� right-hand side �� B.
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
