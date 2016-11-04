#ifndef _REC_ROBOTINO3_SERIALIO_PARSER_H_
#define _REC_ROBOTINO3_SERIALIO_PARSER_H_

#include <QtCore>
#include "rec/robotino3/serialio/Tag.h"

#include <exception>
#include <string>

namespace rec
{
	namespace serialport
	{
		class SerialPort;
	}

	namespace robotino3
	{
		namespace serialio
		{
			class Decoder;

			class ParserDelegate
			{
			public:
				ParserDelegate()
				{
				}

				virtual ~ParserDelegate()
				{
				}

				virtual int parserDelegateVerbosity() const { return 0; };
				virtual void parserDelegateLog( const QString& message ) {};
			};

			class Parser
			{
			public:
				Parser( rec::serialport::SerialPort* serial, Decoder* decoder, ParserDelegate* delegate );

				TagList parse();

			private:
				QByteArray read_unmangled( int length ) const;
				quint16 readUInt16() const;

				rec::serialport::SerialPort* _serial;
				Decoder* _decoder;
				ParserDelegate* _delegate;
			};

			class ParserException : public std::exception
			{
			public:
				ParserException( const QString& message )
					: msg( message.toStdString() )
				{
				}

				virtual ~ParserException() throw ()
				{
				}

				virtual const std::string& getMessage() const
				{
					return msg;
				}

				//Compatibility functions for std::exception
				virtual const char* what() const throw ()
				{
					return msg.c_str();
				}

			protected:
				std::string msg;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_PARSER_H_
