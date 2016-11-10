#ifndef _REC_NSTAR2_Parser_H_
#define _REC_NSTAR2_Parser_H_

#include <QtCore>
#include "rec/nstar2/types.h"
#include "rec/nstar2/tag/Tag.h"

#include <exception>
#include <string>

namespace rec
{
	namespace serialport
	{
		class SerialPort;
	}

	namespace nstar2
	{
		class Com;

		class Parser
		{
		public:
			Parser( rec::serialport::SerialPort* serial, Com* com );

			tag::TagList parse();

		private:
			quint16 calculate_checksum( const QByteArray& body ) const;
			QByteArray read_unmangled( int length ) const;
			quint16 readUInt16() const;

			rec::serialport::SerialPort* _serial;
			Com* _com;
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

#endif //_REC_NSTAR2_Parser_H_
