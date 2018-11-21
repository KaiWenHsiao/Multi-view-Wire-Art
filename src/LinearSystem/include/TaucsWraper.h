#ifndef _TAUCS_WRAPER_H_
#define _TAUCS_WRAPER_H_

#include "taucs_all.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <cassert>
#include <windows.h>

using namespace std;

namespace LinearSystemLib
{
	namespace TaucsLib
	{
		static char g_filename[256];

		class Taucs
		{
		private:
			Taucs() {;}

		public:

			//-----------------------------------------------------------------------------
			// LLT 解法。

			static taucs_ccs_matrix* FactorToLLT( taucs_ccs_matrix* A, double droptol, int modified );
			static int SolveLLT( void* L, double* x, double* b );

			//-----------------------------------------------------------------------------
			// Creating and deleting sparse matrices.

			static taucs_ccs_matrix* Create( int m, int n, int nnz, int flags );

			static void Free( taucs_ccs_matrix* A );
			static void Free( void* A );

			static taucs_ccs_matrix* CloneMatrix( const taucs_ccs_matrix* A );

			//-----------------------------------------------------------------------------
			// reading and writing sparse matrices.
			static taucs_ccs_matrix* ReadMatrixIjv( const char* filename, int flags );

			static int WriteMatrixIjv( taucs_ccs_matrix* A, const char* filename );

			static taucs_ccs_matrix* ReadMatrixMtx( const char* filename, int flags );

			//static int WriteMatrixMtx( taucs_ccs_matrix* A, const char* filename );

			static taucs_ccs_matrix* ReadMatrixCcs( const char* filename, int flags );

			//static int WriteMatrixCcs( taucs_ccs_matrix* A, const char* filename );

			static taucs_ccs_matrix* ReadMatrixBinary( const char* filename );

			static taucs_ccs_matrix* ReadMatrixHb( const char* filename, int flags );

			// layan extends functions.

			static void WriteMatrixIjv( ostream& _out, const taucs_ccs_matrix* A );

			//-----------------------------------------------------------------------------
			// vectors.

			static void MatrixMultiplyVector( taucs_ccs_matrix* A, void* x, void* b );

			static void* ReadVecBinary( int n, int flags, const char* filename );

			static int WriteVecBinary( int n, int flags, void* v, const char* filename );

			//-----------------------------------------------------------------------------
			// utility routines.

			// 暫時勿用此函式，在 managed C++ 下用此函式好像會錯.
			static void LogFile( const char* file_prefix );

			static double SystemMemorySize();
			static double AvailableMemorySize();

			static double Wtime();
			static double CTime();

			//-----------------------------------------------------------------------------
			// Unified Linear Solver.

			// note: 可分三次做，
			// 1. factor.
			// 2. solve.
			// 3. release factor.

			// 對 A 做 Factorization 並存在 F.
			static int LinSolveLLTFactor( taucs_ccs_matrix* A, void* &F );

			static int LinSolve( taucs_ccs_matrix* A, void** factorization, int nrhs,
				void* x, void* b, char* options[], void* arguments[] );

			// 釋放 factorization F.
			static int LinSolveFree( void* &F );

			//-----------------------------------------------------------------------------
			// helper functions.
			static void ErrorReport( int error );
		};

		inline taucs_ccs_matrix* Taucs::FactorToLLT( taucs_ccs_matrix* A, double droptol, int modified )
		{
			return taucs_ccs_factor_llt( A, droptol, modified );
		}

		inline int Taucs::SolveLLT( void* L, double* x, double* b )
		{
			return taucs_ccs_solve_llt( L, x, b );
		}

		//-----------------------------------------------------------------------------
		// Creating and deleting sparse matrices.

		inline taucs_ccs_matrix* Taucs::Create( int m, int n, int nnz, int flags )
		{
			return taucs_ccs_create( m, n, nnz, flags );
		}

		inline void Taucs::Free( taucs_ccs_matrix* A )
		{
			taucs_ccs_free(A);
		}

		inline void Taucs::Free( void* A )
		{
			taucs_ccs_free( reinterpret_cast<taucs_ccs_matrix*>(A) );
		}

		//-----------------------------------------------------------------------------
		// reading and writing sparse matrices.
		inline taucs_ccs_matrix* Taucs::ReadMatrixIjv( const char* filename, int flags )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_read_ijv( g_filename, flags );
		}

		inline int Taucs::WriteMatrixIjv( taucs_ccs_matrix* A, const char* filename )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_write_ijv( A, g_filename );
		}

		inline taucs_ccs_matrix* Taucs::ReadMatrixMtx( const char* filename, int flags )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_read_mtx( g_filename, flags );
		}

		/*int Taucs::WriteMatrixMtx( taucs_ccs_matrix* A, const char* filename )
		{
		strcpy_s(g_filename, 298, filename);
		return taucs_ccs_write_mtx( A, g_filename );
		}*/

		inline taucs_ccs_matrix* Taucs::ReadMatrixCcs( const char* filename, int flags )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_read_ccs( g_filename, flags );
		}

		/*int Taucs::WriteMatrixCcs( taucs_ccs_matrix* A, const char* filename )
		{
		strcpy_s(g_filename, 298, filename);
		return taucs_ccs_write_ccs( A, g_filename );
		}*/

		inline taucs_ccs_matrix* Taucs::ReadMatrixBinary( const char* filename )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_read_binary( g_filename );
		}

		inline taucs_ccs_matrix* Taucs::ReadMatrixHb( const char* filename, int flags )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_ccs_read_hb( g_filename, flags );
		}

		//-----------------------------------------------------------------------------
		// vectors.

		inline void Taucs::MatrixMultiplyVector( taucs_ccs_matrix* A, void* x, void* b )
		{
			taucs_ccs_times_vec( A, x, b );
		}

		inline void* Taucs::ReadVecBinary( int n, int flags, const char* filename )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_vec_read_binary( n, flags, g_filename );
		}

		inline int Taucs::WriteVecBinary( int n, int flags, void* v, const char* filename )
		{
			strcpy_s(g_filename, 298, filename);
			return taucs_vec_write_binary( n, flags, v, g_filename );
		}

		//-----------------------------------------------------------------------------
		// utility routines.

		inline void Taucs::LogFile( const char* file_prefix )
		{
			// 暫時勿用此函式，在 managed C++ 下用此函式好像會錯.
			assert( false );
			strcpy_s( g_filename, 298, file_prefix );
			taucs_logfile( g_filename );
		}

		inline double Taucs::SystemMemorySize()
		{
			return taucs_system_memory_size();
		}

		inline double Taucs::AvailableMemorySize()
		{
			return taucs_available_memory_size();
		}

		inline double Taucs::Wtime()
		{
			return taucs_wtime();
		}

		inline double Taucs::CTime()
		{
			return taucs_ctime();
		}

		//-----------------------------------------------------------------------------
		// Unified Linear Solver.

		inline int Taucs::LinSolveLLTFactor( taucs_ccs_matrix* A, void* &F )
		{
			char* factor_opt[] = { "taucs.factor.LLT=true", NULL };
			return taucs_linsolve( A, &F, 0, NULL, NULL, factor_opt, NULL );
		}

		inline int Taucs::LinSolve( taucs_ccs_matrix* A, void** factorization, int nrhs,
			void* x, void* b, char* options[], void* arguments[] )
		{
			return taucs_linsolve( A, factorization, nrhs, x, b, options, arguments );
		}

		inline int Taucs::LinSolveFree( void* &F )
		{
			return taucs_linsolve( NULL, &F, 0, NULL, NULL, NULL, NULL );
		}
	}

}

#endif