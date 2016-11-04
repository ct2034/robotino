#ifndef _REC_EA09_MESSAGES_POWEROFF_H_
#define _REC_EA09_MESSAGES_POWEROFF_H_

#include "rec/ea09/messages/util.h"

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class PowerOff
			{
			public:
				static void encode( unsigned char* buffer )
				{
					buffer[0] = 28;
					buffer[1] = 3;
					buffer[2] = 0;

					buffer[2] = generate_checkSum( buffer );
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_POWEROFF_H_
