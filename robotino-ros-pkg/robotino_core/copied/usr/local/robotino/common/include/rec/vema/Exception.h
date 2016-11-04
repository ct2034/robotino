#ifndef REC_VEMA_EXCEPTION_H
#define REC_VEMA_EXCEPTION_H

#include <exception>
#include <string>

namespace rec
{
	namespace vema
	{

		/**
		* The base class of all exceptions provided by the VEMA PCB class.
		*/
		class Exception : public std::exception
		{
		protected:
			std::string msg;

		public:
			Exception( const std::string &message )
				: msg( message )
			{
			}

			virtual ~Exception() throw ()
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
		};

	}
}

#endif	// REC_VEMA_EXCEPTION_H
