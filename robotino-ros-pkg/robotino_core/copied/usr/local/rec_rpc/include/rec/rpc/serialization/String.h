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

#ifndef _REC_RPC_SERIALIZATION_STRING_H_
#define _REC_RPC_SERIALIZATION_STRING_H_

#include "rec/rpc/serialization/Primitive.h"
#include "rec/rpc/defines.h"

#include <QString>

namespace rec
{
	namespace rpc
	{
		namespace serialization
		{
			/*! \brief Serialization class for strings. */
			class REC_RPC_EXPORT String : public Primitive< QString >
			{
			public:
				/*! Creates a container storing an empty string. */
				String();
				/*! Creates a container storing s. */
				String( const QString& s );
				/*! Converts s to a QString and creates a container storing it. */
				String( const std::string& s );
				/*! Converts s to a QString and creates a container storing it. */
				String( const std::wstring& s );
				/*! Converts s to a QString and creates a container storing it. */
				String( const char* s );
				/*! Converts s to a QString and creates a container storing it. */
				String( const wchar_t* s );

				/*! \brief Convert the stored string to a std::string */
				std::string toStdString() const;
				/*! \brief Modify the stored string with a std::string. */
				void setStdString( const std::string& s );

				/*! \brief Convert the stored string to a std::wstring */
				std::wstring toStdWString() const;
				/*! \brief Modify the stored string with a std::wstring. */
				void setStdWString( const std::wstring& s );

				/*! \brief Modify the stored string with a C string. */
				void setCString( const char* s );
				/*! \brief Modify the stored string with a C wide character string. */
				void setCWString( const wchar_t* s );
			};
		}
	}
}

/*!
 *  \brief Complex data container member definition for strings.
 *
 *  Place this macro into the declaration of a rec::rpc::serialization::Complex subclass to add a string.
 *  It can be accessed via NAME() (const and non-const).
 *
 *  \param NAME Member name.
 *
 *  \sa rec::rpc::serialization::Complex
 */
#define DECLARE_STRING_MEMBER( NAME ) \
	private: \
		QSharedPointer< rec::rpc::serialization::String > _##NAME; \
		void create_##NAME() { _##NAME = QSharedPointer< rec::rpc::serialization::String >( new rec::rpc::serialization::String ); } \
	public: \
		const QString& NAME() const { return *_##NAME; } \
		QString& NAME() { return *_##NAME; } \
		void set##NAME( const QString& value ) { _##NAME->setValue( value ); }

/*!
 *  \brief Function parameter type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_STRING_PARAM( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Param, rec::rpc::serialization::String );

/*!
 *  \brief Function result type definition.
 *
 *  \param FUNCTIONNAME Name of the RPC function.
 */
#define DEFINE_STRING_RESULT( FUNCTIONNAME ) DEFINE_SERIALIZABLE( FUNCTIONNAME##Result, rec::rpc::serialization::String );

/*!
 *  \brief Topic data type definition.
 *
 *  \param TOPICNAME Name of the topic.
 */
#define DEFINE_STRING_TOPICDATA( TOPICNAME ) DEFINE_SERIALIZABLE( topic##TOPICNAME##Data, rec::rpc::serialization::String );

#endif //_REC_RPC_SERIALIZATION_STRING_H_
