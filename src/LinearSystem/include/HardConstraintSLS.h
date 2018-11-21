#ifndef _HARD_CONSTRAINT_SPARSE_LINEAR_SYSTEM_H_
#define _HARD_CONSTRAINT_SPARSE_LINEAR_SYSTEM_H_

#include "SparseLinearSystem.h"

namespace LinearSystemLib
{
	/// ConType : constraint 的 type, ex : Vi = pi, 其中 pi 是 Point3f Type.
	template < class ConType >
	class ConstraintContainer : public map< unsigned int, ConType const* > 
	{
	public:
		unsigned int Index( const const_iterator& cit ) const	{ return cit->first; }
		unsigned int Index( const iterator& it ) const			{ return it->first; }
		const ConType* Value( const_iterator& cit ) const		{ return cit->second; }
		ConType const*& Value( iterator& cit )					{ return cit->second; }
	};

	template < class ConType, class Extractor >
	class HardConstraintSLSBuilder;

	template < class ConType >
	class HardConstraintSLS : public SparseLinearSystem
	{
		template < class ConType, class Extractor >
		friend class HardConstraintSLSBuilder;

	private:
		// 不支援 copy constructor.
		HardConstraintSLS( const HardConstraintSLS& );

	public:
		HardConstraintSLS( unsigned int dim = 0 )
			: SparseLinearSystem( 0, 0, dim )
		{}

		const ConstraintContainer<ConType>& Constraints() const 
		{ 
			return mc_Constraints; 
		}

	protected:

		void setConstraints( const ConstraintContainer<ConType>& cons )
		{
			mc_Constraints.erase( mc_Constraints.begin(), mc_Constraints.end() );
			mc_Constraints.insert( cons.begin(), cons.end() );
		}

		void setConstraint( unsigned int index, ConType const* value )
		{
			ConstraintContainer<ConType>::iterator it = mc_Constraints.find( index );

			if( it != mc_Constraints.end() )
				mc_Constraints.Value(it) = value;
			else
				mc_Constraints.insert( make_pair(index, value) );
		}	

	private:

		ConstraintContainer<ConType> mc_Constraints;
	};
}

#endif