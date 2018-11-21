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
		// n : 所有變數的個數，可包含未知解個數+己知解個數或是只包含未知解個數。
		// dim : 解的維度，若在 3D 上則 dim = 3. etc.
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
		/// 若使用完畢必須要 自行delete 掉此 linear system.
		LinearSystem* GetResult() { return mp_LinearSystem; }

	protected:

		LinearSystem*& linearSystem()				{ return mp_LinearSystem; }
		const LinearSystem* linearSystem() const	{ return mp_LinearSystem; }

		unsigned int& numVariables()		{ return mui_TotalVars; }
		unsigned int numVariables() const	{ return mui_TotalVars; }

		unsigned int& dimension()			{ return mui_Dim; }
		unsigned int dimension() const		{ return mui_Dim; }

	private:

		/// 建出來的 linear system 必須由使用者 deallocate, 因為 builder 只負責
		/// 定義怎麼建造 linear system.
		LinearSystem* mp_LinearSystem;

		// 所有變數的個數，可包含未知解個數+己知解個數或是只包含未知解個數。
		unsigned int mui_TotalVars;

		// B 的維度, 若 3D 的話會有 {Bx, By, Bz} => dim = 3.
		unsigned int mui_Dim;
	};
}


#endif