#ifndef _REC_EA09_MESSAGES_GETINFORMATION_H_
#define _REC_EA09_MESSAGES_GETINFORMATION_H_

#include "rec/ea09/messages/util.h"

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class GetInformation
			{
			public:
				static void encode( unsigned char* buffer )
				{
					buffer[0] = 20;
					buffer[1] = 3;
					buffer[2] = 0;

					buffer[2] = generate_checkSum( buffer );
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_GETINFORMATION_H_
