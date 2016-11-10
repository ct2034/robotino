//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _REC_SERIALPORT_SCANNEREXCEPTION_H_
#define _REC_SERIALPORT_SCANNEREXCEPTION_H_

#include <exception>
#include <string>

namespace rec
{
	namespace laserrangefinder
	{
		class ScannerException : public std::exception
		{
		public:
			ScannerException( const std::string &message )
				: msg( message )
			{
			}

			virtual ~ScannerException() throw ()
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

#endif //_REC_SERIALPORT_SCANNEREXCEPTION_H_
