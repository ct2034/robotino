//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _REC_SERIALPORT_SERIALPORTEXCEPTION_H_
#define _REC_SERIALPORT_SERIALPORTEXCEPTION_H_

#include <exception>
#include <string>

namespace rec
{
	namespace serialport
	{
		class SerialPortException : public std::exception
		{
		public:
			SerialPortException( const std::string &message )
				: msg( message )
			{
			}

			virtual ~SerialPortException() throw ()
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

#endif
