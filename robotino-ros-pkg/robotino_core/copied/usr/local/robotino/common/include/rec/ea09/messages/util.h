#ifndef _REC_EA09_MESSAGES_UTIL_H_
#define _REC_EA09_MESSAGES_UTIL_H_

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			bool isMessageCorrect( const unsigned char* message );

			unsigned char generate_checkSum( const unsigned char* message );

			bool isCheckSumCorrect( const unsigned char* message );
		}
	}
}

#endif //_REC_EA09_MESSAGES_UTIL_H_

