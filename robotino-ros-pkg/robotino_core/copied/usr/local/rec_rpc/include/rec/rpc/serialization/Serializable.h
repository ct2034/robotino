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

#ifndef _REC_RPC_SERIALIZATION_SERIALIZABLE_H_
#define _REC_RPC_SERIALIZATION_SERIALIZABLE_H_

#include <QSharedPointer>
#include <QDataStream>
#include <QMetaType>
#include <typeinfo>

#include "rec/rpc/defines.h"

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*! \cond */
			class Serializable;
			typedef QSharedPointer< Serializable > SerializablePtr;
			typedef QSharedPointer< const Serializable > SerializablePtrConst;
			/*! \endcond */

			/*!
			 *  \brief Base class for all serializable data.
			 *
			 *  This class is the base class for all serializable data container types. All function parameter, return value and topic data types must inherit this class.
			 */
			class REC_RPC_EXPORT Serializable
			{
				friend QDataStream& operator<<( QDataStream&, const Serializable& );
				friend QDataStream& operator>>( QDataStream&, Serializable& );

			public:
				/*! Empty serializable. */
				static SerializablePtr empty;

				/*! Creates an empty container. */
				Serializable();
				/*! Destroys the container. */
				virtual ~Serializable();

			protected:
				/*!
				 *  In a subclass, this function must be reimplemented to write the data into the stream.
				 *
				 *  \param stream Data stream that serializes the data.
				 *
				 *  \sa deserialize()
				 */
				virtual void serialize( QDataStream& stream ) const;

				/*!
				 *  In a subclass, this function must be reimplemented to read data from the stream.
				 *
				 *  \param stream Data stream that contains and deserializes the data.
				 *
				 *  \sa serialize()
				 */
				virtual void deserialize( QDataStream& stream );
			};

			inline Serializable::Serializable()
			{
			}

			inline Serializable::~Serializable()
			{
			}

			inline void Serializable::serialize( QDataStream& ) const
			{
			}

			inline void Serializable::deserialize( QDataStream& )
			{
			}

			/*!
			 *  \brief Serialize the data and write it into a data stream.
			 *
			 *  \param out Data stream.
			 *  \param data Data to be serialized. The serialize() method will be used.
			 *
			 *  \sa operator>>(), Serializable::serialize(), Serializable::deserialize()
			 */
			inline QDataStream& operator<<( QDataStream& out, const Serializable& data )
			{
				data.serialize( out );
				return out;
			}

			/*!
			 *  \brief Read data from a data stream and deserialize it.
			 *
			 *  \param in Data stream.
			 *  \param data Object that stores the deserialized data. The deserialize() method will be used.
			 *
			 *  \sa operator<<(), Serializable::serialize(), Serializable::deserialize()
			 */
			inline QDataStream& operator>>( QDataStream& in, Serializable& data )
			{
				data.deserialize( in );
				return in;
			}
		}
	}
}

Q_DECLARE_METATYPE( rec::rpc::serialization::SerializablePtr );
Q_DECLARE_METATYPE( rec::rpc::serialization::SerializablePtrConst );

/*! \cond */
#define DEFINE_SERIALIZABLE( NAME, TYPE ) \
	typedef TYPE NAME; \
	typedef QSharedPointer< NAME > NAME##Ptr; \
	typedef QSharedPointer< const NAME > NAME##PtrConst;
/*! \endcond */

/*!
 *  \brief Empty function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_EMPTY_PARAM( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::Serializable );

/*!
 *  \brief Empty function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_EMPTY_RESULT( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::Serializable );

/*!
 *  \brief Empty topic data type definition.
 *
 *  \param TOPICNAME Name of the RPC function.
 */
#define DEFINE_EMPTY_TOPICDATA( TOPICNAME ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::Serializable );

#endif //_REC_RPC_SERIALIZATION_SERIALIZABLE_H_
