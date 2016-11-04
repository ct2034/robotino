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

#ifndef _REC_RPC_EXCEPTION_H_
#define _REC_RPC_EXCEPTION_H_

#include "rec/rpc/defines.h"

#include <QCoreApplication>
#include <QString>
#include <QMetaType>
#include <exception>

namespace rec
{
	namespace rpc
	{
		/*! \brief Pre-defined error codes. */
		enum ErrorCode
		{
			NoError = 0,			/*!< No error. */

			NoConnection,			/*!< Connection was lost or could not been established. */
			NotAnRPCServer,			/*!< The server the client is trying to connect to is not a RPC server. */
			IncompatibleServer,		/*!< The server the client is trying to connect to is incompatible (because it sent a greeting that is different from the one expected. */
			UnknownFunction,		/*!< The function the client is trying to call does not exist on the server. */
			ExecutionCancelled,		/*!< The function execution on the server has been cancelled for unknown reasons. */
			ExecutionTimeout,		/*!< A timeout during function execution has occurred. */
			WrongDataFormat,		/*!< RPC function parameters or return values or topic data have an incompativle data type or format. */
			NoSuchTopic,			/*!< The topic the client or server is trying to access does not exist. */
			ImproperTopicName,		/*!< The topic name specified in ADD_TOPIC is not permitted. */
			TopicAlreadyExists,		/*!< A topic with the name specified in ADD_TOPIC already exists. */
			AccessDenied,			/*!< A client is trying to publish data on a server-only topic. */
			ImproperFunctionName,	/*!< The function name specified in REGISTER_FUNCTION is not permitted. */
			LocalTopicNull,			/*!< The data in the local shared memory is Null*/ 

			HTTPNotFount = 100,		/*!< The page that was requested via HTTP does not exist. */
			HTTPBadRequest,			/*!< The browser sent an invalid HTTP request. */
			HTTPMovedPermanently,	/*!< The page requestet via HTTP has moved to a new location. The new location can be retrieved by calling detail(). */

			UnknownError,			/*!< An unknown error occurred. */

			User = 200,
		};

		/*! \brief Exception class */
		class REC_RPC_EXPORT Exception : public std::exception
		{
			Q_DECLARE_TR_FUNCTIONS( rec::rpc::Exception );

		public:
			/*! \brief Constructor.
			 *
			 *  \param code Error code.
			 *  \param detail Detailled human readable error message.
			 */
			Exception( ErrorCode code = UnknownError, const QString& detail = QString::null );
			/*! \brief Destructor. */
			virtual ~Exception() throw();

			/*! \return Error code. */
			ErrorCode errorCode() const;
			/*! \return Human readable error message. */
			QString getMessage() const;
			/*! \return The detailled error message only (if any). */
			QString detail() const;

			/*!
			 *  For std::exception compatibility.
			 *
			 *  \return Error message as C string.
			 */
			virtual const char* what() const throw();

			/*!
			 *  \brief Error message from error code.
			 *
			 *  \param code Error code.
			 *
			 *  \return Error message.
			 */
			static QString messageFromErrorCode( ErrorCode code );

		private:
			ErrorCode _code;
			mutable QByteArray _msg;
			const QString _detail;
		};
	}
}

Q_DECLARE_METATYPE( rec::rpc::ErrorCode );

#endif //_REC_RPC_EXCEPTION_H_
