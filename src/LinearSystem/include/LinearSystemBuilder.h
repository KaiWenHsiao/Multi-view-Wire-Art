#ifndef _LINEAR_SYSTEM_BUILDER_H_
#define _LINEAR_SYSTEM_BUILDER_H_

#include "LinearSystem.h"

namespace LinearSystemLib
{
	//===============================================================================================

	/// the linear system builder interface.
	class LinearSystemBuilder
	{
		LinearSystemBuilder( const LinearSystemBuilder& );
		LinearSystemBuilder& operator= ( const LinearSystemBuilder& );
	
	protected:

		// constructor.
		// n : �Ҧ��ܼƪ��ӼơA�i�]�t�����ѭӼ�+�v���ѭӼƩάO�u�]�t�����ѭӼơC
		// dim : �Ѫ����סA�Y�b 3D �W�h dim = 3. etc.
		LinearSystemBuilder( unsigned int n = 0, unsigned int dim = 0 )
			: mp_LinearSystem(0)
			, mui_TotalVars( n )
			, mui_Dim( dim )
		{}

	public:
		/// destructor.
		virtual ~LinearSystemBuilder() {;}

		/// define the way to build the linear system.
		/// define by user.
		virtual void BuildLinearSystem() = 0;

		/// get the final linear system.
		/// �Y�ϥΧ��������n �ۦ�delete ���� linear system.
		LinearSystem* GetResult() { return mp_LinearSystem; }

	protected:

		LinearSystem*& linearSystem()				{ return mp_LinearSystem; }
		const LinearSystem* linearSystem() const	{ return mp_LinearSystem; }

		unsigned int& numVariables()		{ return mui_TotalVars; }
		unsigned int numVariables() const	{ return mui_TotalVars; }

		unsigned int& dimension()			{ return mui_Dim; }
		unsigned int dimension() const		{ return mui_Dim; }

	private:

		/// �إX�Ӫ� linear system �����ѨϥΪ� deallocate, �]�� builder �u�t�d
		/// �w�q���سy linear system.
		LinearSystem* mp_LinearSystem;

		// �Ҧ��ܼƪ��ӼơA�i�]�t�����ѭӼ�+�v���ѭӼƩάO�u�]�t�����ѭӼơC
		unsigned int mui_TotalVars;

		// B ������, �Y 3D ���ܷ|�� {Bx, By, Bz} => dim = 3.
		unsigned int mui_Dim;
	};
}


#endif