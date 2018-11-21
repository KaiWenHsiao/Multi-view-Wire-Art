#ifndef _LINEARSYSTEMLIB_SINGLETON_H_
#define _LINEARSYSTEMLIB_SINGLETON_H_

#include <cassert>

namespace LinearSystemLib 
{
    // Template class for creating single-instance global classes.

    template < class _Type > class Singleton
    {
    public:
        Singleton();
		virtual  ~Singleton();
        static _Type& GetInstance();
        static _Type* GetInstancePtr();   
	
	protected:
        static _Type* ms_Singleton;
    };

	template < class _Type >
		Singleton<_Type>::Singleton()
	{
		assert( !ms_Singleton );
		ms_Singleton = static_cast< _Type* >( this );
	}

	template < class _Type >
		Singleton<_Type>::~Singleton()
	{  
		assert( ms_Singleton ); 
		ms_Singleton = 0;  
	}

	template < class _Type >
		_Type& Singleton<_Type>::GetInstance()
	{	
		assert( ms_Singleton );  
		return *ms_Singleton; 
	}

	template < class _Type >
		_Type* Singleton<_Type>::GetInstancePtr()
	{ 
		return ms_Singleton; 
	}
}

#endif