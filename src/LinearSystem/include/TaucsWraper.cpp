#include "TaucsWraper.h"

namespace LinearSystemLib
{
	namespace TaucsLib
	{
		//-----------------------------------------------------------------------------
		// Creating and deleting sparse matrices.

		taucs_ccs_matrix* Taucs::CloneMatrix( const taucs_ccs_matrix* A )
		{
			int nnz =  A->colptr[A->n];
			taucs_ccs_matrix* obj = taucs_ccs_create( A->m, A->n, nnz, A->flags );

			if( obj == NULL )
				throw exception( "call taucs_ccs_create failed - in Taucs::CloneMatrix." );

			memcpy( obj->colptr, A->colptr, sizeof(int) * (A->n + 1) );
			memcpy( obj->rowind, A->rowind, sizeof(int) * nnz );
			memcpy( obj->values.d, A->values.d, sizeof(double) * nnz );

			return obj;
		}

		//-----------------------------------------------------------------------------
		// reading and writing sparse matrices.

		void Taucs::WriteMatrixIjv( ostream& _out, const taucs_ccs_matrix* _m )
		{
			int i, ip, j, n;
			double Aij;

			n = _m->n;

			// enable this if you want.
			//_out.setf(ios::scientific,ios::floatfield);
			//_out.setf(ios::fixed,ios::floatfield);
			//_out.precision(17);

			for( j = 0 ; j < n ; ++j )
			{
				for( ip = (_m->colptr)[j]; ip < (_m->colptr[j+1]) ; ++ip ) 
				{
					i   = (_m->rowind)[ip];
					Aij = (_m->values.d)[ip];

					_out << i+1 << " " << j+1 << " " << Aij << endl;
					if( i != j && _m->flags & TAUCS_SYMMETRIC )
						_out << j+1 << " " << i+1 << " " << Aij << endl;
				}
			}
		}
		
		//-----------------------------------------------------------------------------
		// helper functions.

		void Taucs::ErrorReport( int error )
		{
			static const char* str_error[6] = 
			{
				"TAUCS_ERROR", "TAUCS_ERROR_NOMEM", "TAUCS_ERROR_BADARGS", 
				"TAUCS_ERROR_MAXDEPTH","TAUCS_ERROR_INDEFINITE", "NON_DEFINE, ASK JIN LONG."
			};

			int idx = -1;

			if( error == TAUCS_ERROR )
				idx = 0;
			else if( error == TAUCS_ERROR_NOMEM )
				idx = 1;
			else if( error == TAUCS_ERROR_BADARGS )
				idx = 2;
			else if( error == TAUCS_ERROR_MAXDEPTH )
				idx = 3;
			else if( error == TAUCS_ERROR_INDEFINITE )
				idx = 4;
			else
				idx = 5;

			//MessageBox( NULL, str_error[idx], "Taucs Error Message Report", MB_OK );
			cout << "Taucs Error Message Report - " << str_error[idx] << endl;
		}
	}

}