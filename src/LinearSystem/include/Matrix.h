#ifndef _MATRIX_H_
#define _MATRIX_H_

#include "Prerequistites.h"
#include <iostream>

namespace LinearSystemLib
{
	using std::ostream;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// forward declaration
	_interface Matrix;
	_interface SparseMatrix;
	_interface SerializeMatrix;
	
	////////////////////////////////////////////////////////////////////////////////////////////////////

	// a strategy pattern.
	_interface SerializeMatrix
	{
		virtual ~SerializeMatrix() {;}
		virtual void Write( ostream& _out, const Matrix* _sm ) const = 0;
	};

	// define interface of Matrix.
	_interface Matrix
	{
		virtual ~Matrix() {;}

		// abstract operations.
		virtual Matrix* Clone() const = 0;
		virtual Matrix* Transpose() const = 0;
		virtual double GetElement( unsigned int _row_index, unsigned int _col_index ) const = 0;

		virtual void Write( ostream& _out, SerializeMatrix* _format ) const = 0;

		virtual unsigned int GetNumRows() const = 0;
		virtual unsigned int GetNumCols() const = 0;
	};

	_interface SparseMatrix : public Matrix
	{
		virtual ~SparseMatrix() {;}

		// Multiplies this matrix by x and stores the result in b. Assumes all memory has been allocated
		// and the sizes match; assumes this matrix is not symmetric!!
		virtual void MultiplyVector( double* _x, double* _b ) const = 0;

		// get the normal equation form, if this matrix is A, it will return A^T * A.
		virtual SparseMatrix* NormalEquationForm() const = 0;

		virtual unsigned int GetNumNonZero() const = 0;

		// factory methods.
		virtual SerializeMatrix* GetSerializeIjv() const = 0;
		virtual SerializeMatrix* GetSerializeFull() const = 0;

		// here must define some iterator for travel all the nonzero elements.
	};
}

#endif