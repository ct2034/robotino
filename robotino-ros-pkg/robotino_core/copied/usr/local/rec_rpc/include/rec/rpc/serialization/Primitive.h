/*
Copyright (c) 2011, REC Robotics Equipment Corporation GmbH, Planegg, Germany
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.
- Neither the name of the REC Robotics Equipment Corporation GmbH nor the names of
  its contributors may be used to endorse or promote products derived from this software
  without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _REC_RPC_SERIALIZATION_PRIMITIVE_H_
#define _REC_RPC_SERIALIZATION_PRIMITIVE_H_

#include "rec/rpc/serialization/Serializable.h"

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*!
			 *  \brief Serialization class for primitive objects and values.
			 *
			 *  Primitive is a template class that provides a serializable container for primitive data types.
			 *  Every type T for which an operator<<( QDataStream&, const T& ) and an operator>>( QDataStream&, T& ) exist is permitted.
			 *  These are usually all numeric types, several Qt classes and types and all Serializable subclasses.
			 */
			template< typename T >
			class REC_RPC_VISIBILITY_DEFAULT Primitive : public Serializable
			{
			public:
				/*! Creates a container storing the default value. */
				Primitive();
				/*! Creates a container storing v. */
				Primitive( const T& v );

				/*!
				 *  \brief Retrieve the stored value.
				 *
				 *  \return The stored value.
				 */
				T value() const;

				/*!
				 *  \brief Modify the stored value.
				 *
				 *  \param value New value.
				 */
				void setValue( const T& value );

				/*!
				 *  \brief Access the stored value.
				 *
				 *  \return A const reference to the stored value.
				 */
				const T& ref() const;

				/*!
				 *  \brief Access the stored value.
				 *
				 *  \return A reference to the stored value.
				 */
				T& ref();

				/*! \brief Type cast operator. */
				operator T&();
				/*! \brief Type cast operator (constant). */
				operator const T&() const;

				/*! \cond */
			protected:
				T _value;
				/*! \endcond */

			private:
				void serialize( QDataStream& ) const;
				void deserialize( QDataStream& );
			};

			template< typename T >
			Primitive< T >::Primitive()
				: _value( T() )
			{
			}

			template< typename T >
			Primitive< T >::Primitive( const T& v )
				: _value( v )
			{
			}

			template< typename T >
			T Primitive< T >::value() const
			{
				return _value;
			}

			template< typename T >
			void Primitive< T >::setValue( const T& value )
			{
				_value = value;
			}

			template< typename T >
			const T& Primitive< T >::ref() const
			{
				return _value;
			}

			template< typename T >
			T& Primitive< T >::ref()
			{
				return _value;
			}

			template< typename T >
			Primitive< T >::operator T&()
			{
				return _value;
			}

			template< typename T >
			Primitive< T >::operator const T&() const
			{
				return _value;
			}

			template< typename T >
			void Primitive< T >::serialize( QDataStream& stream ) const
			{
				stream << _value;
			}

			template< typename T >
			void Primitive< T >::deserialize( QDataStream& stream )
			{
				stream >> _value;
			}

			/*! \cond */
			template< typename T >
			struct PrimitivePtr
			{
				typedef QSharedPointer< Primitive< T > > Type;
			};
			/*! \endcond */
		}
	}
}

/*!
 *  \brief Complex data container member definition for primitive types.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add a primitive member.
 *  It can be accessed via NAME() (const and non-const).
 *
 *  \param DATATYPE Type of the data contained.
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_PRIMITIVE_MEMBER( DATATYPE, NAME ) \
	private: \
		rec::rpc::serialization::PrimitivePtr< DATATYPE >::Type _##NAME; \
		void create_##NAME() { _##NAME = rec::rpc::serialization::PrimitivePtr< DATATYPE >::Type( new rec::rpc::serialization::Primitive< DATATYPE > ); } \
	public: \
		DATATYPE& NAME() { return *_##NAME; } \
		const DATATYPE& NAME() const { return *_##NAME; } \
		void set##NAME( const DATATYPE& value ) { _##NAME->setValue( value ); }

/*!
 *  \brief Function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 *  \param DATATYPE Type of the data stored in the container.
 */
#define DEFINE_PRIMITIVE_PARAM( FUNCTIONNAME, DATATYPE ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::Primitive< DATATYPE > );

/*!
 *  \brief Function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 *  \param DATATYPE Type of the data stored in the container.
 */
#define DEFINE_PRIMITIVE_RESULT( FUNCTIONNAME, DATATYPE ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::Primitive< DATATYPE > );

/*!
 *  \brief Topic data type definition.
 *
 *  \param TOPICNAME Name of the topic.
 *  \param DATATYPE Type of the data stored in the container.
 */
#define DEFINE_PRIMITIVE_TOPICDATA( TOPICNAME, DATATYPE ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::Primitive< DATATYPE > );

#endif //_REC_RPC_SERIALIZATION_PRIMITIVE_H_
