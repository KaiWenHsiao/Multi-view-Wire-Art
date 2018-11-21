#ifndef _LINEAR_SYSTEM_LIB_H_
#define _LINEAR_SYSTEM_LIB_H_

#include "Matrix.h"
#include "GeneralSparseMatrix.h"
#include "StableSparseMatrix.h"

#include "LinearSystem.h"
#include "SparseLinearSystem.h"

#include "LinearSystemSolver.h"
#include "SparseLinearSystemManager.h"

#include "LinearSystemBuilder.h"
#include "HardConstraintSLS.h"
#include "HardConstraintSLSBuilder.h"
#include "HardConstraintSLSBuilderSpec.h"

#ifdef _DEBUG
	#ifdef _USE_STLPORT
		#pragma comment( lib, "LinearSystemLibD_static_stlport.lib" )
		#pragma message( "Automatically linking with LinearSystemLibD_static_stlport.lib." )
	#else
		//#pragma comment( lib, "LinearSystemLibD_static.lib" )
		//#pragma message( "Automatically linking with LinearSystemLibD_static.lib." )
		//#pragma comment( lib, "LinearSystemLibD_MD.lib" )
		#pragma comment( lib, "LinearSystemLib_vs2013.lib" )
		#pragma message( "Automatically linking with LinearSystemLibD_MD.lib." )		
	#endif
#endif

#ifdef NDEBUG
	#ifdef _USE_STLPORT
		#pragma comment( lib, "LinearSystemLib_static_stlport.lib" )
		#pragma message( "Automatically linking with LinearSystemLib_static_stlport.lib." )
	#else
		//#pragma comment( lib, "LinearSystemLib_static.lib" )
		//#pragma message( "Automatically linking with LinearSystemLib_static.lib." )
		#pragma comment( lib, "LinearSystemLib_MD.lib" )
		#pragma message( "Automatically linking with LinearSystemLib_MD.lib." )
	#endif
#endif


#endif