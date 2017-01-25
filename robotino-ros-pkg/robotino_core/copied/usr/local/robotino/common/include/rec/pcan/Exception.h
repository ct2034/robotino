#ifndef REC_PCAN_PCAN_EXCEPTION_H
#define REC_PCAN_PCAN_EXCEPTION_H

#include <exception>
#include <string>

namespace rec
{
	namespace pcan
	{

		/**
		* The base class of all exceptions related to CAN bus errors.
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

#endif	// REC_PCAN_PCAN_EXCEPTION_H
