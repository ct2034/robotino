#ifndef _REC_DATAEXCHANGE_UDP_MESSAGE_HPP_
#define _REC_DATAEXCHANGE_UDP_MESSAGE_HPP_

#include "rec/dataexchange_lt/defines.h"

namespace rec
{
	namespace dataexchange_lt
	{
		namespace udp
		{
			/**
			Calculates the message checksum. Make sure to initialize message byte 3 with 0.
			@return Checksum
			*/
			REC_DATAEXCHANGE_EXPORT unsigned char calculateChecksum( const char* message, unsigned int messageSize );
			
			/**
			Checks if the message checksum is correct.
			@return Returns true if message is correct. Returns false otherwise.
			*/
			REC_DATAEXCHANGE_EXPORT bool isMessageCorrect( const char* message, unsigned int messageSize );

			/**
			Write value to buffer.
			@return Returns the buffer+sizeof(uint16).
			*/
			REC_DATAEXCHANGE_EXPORT char* writeUInt16( char* buffer, unsigned short value );

			/**
			Write value to buffer.
			@return Returns the buffer+sizeof(int32).
			*/
			REC_DATAEXCHANGE_EXPORT char* writeInt32( char* buffer, int value );

			/**
			Read value from buffer.
			@return Returns the buffer+sizeof(uint16).
			*/
			REC_DATAEXCHANGE_EXPORT const char* readUInt16( const char* buffer, unsigned short* value );

			/**
			Read value from buffer.
			@return Returns the buffer+sizeof(int32).
			*/
			REC_DATAEXCHANGE_EXPORT const char* readInt32( const char* buffer, int* value );
		}
	}
}

#endif //_REC_DATAEXCHANGE_UDP_MESSAGE_HPP_
