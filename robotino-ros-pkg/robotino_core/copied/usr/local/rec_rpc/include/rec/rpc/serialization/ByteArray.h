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

#ifndef _REC_RPC_SERIALIZATION_BYTEARRAY_H_
#define _REC_RPC_SERIALIZATION_BYTEARRAY_H_

#include "rec/rpc/serialization/Primitive.h"

#include <QByteArray>

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*! \brief Pre-defined container type for QByteArray. */
			typedef Primitive< QByteArray > ByteArray;
		}
	}
}

/*!
 *  \brief Complex data container member definition for byte arrays.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add a byte array.
 *  It can be accessed via NAME() (const and non-const).
 *
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_BYTEARRAY_MEMBER( NAME ) DECLARE_PRIMITIVE_MEMBER( QByteArray, NAME )

/*!
 *  \brief Function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_BYTEARRAY_PARAM( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::ByteArray );

/*!
 *  \brief Function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_BYTEARRAY_RESULT( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::ByteArray );

/*!
 *  \brief Topic data type definition.
 *
 *  \param TOPICNAME Name of the topic.
 */
#define DEFINE_BYTEARRAY_TOPICDATA( TOPICNAME ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::ByteArray );

#endif //_REC_RPC_SERIALIZATION_BYTEARRAY_H_
