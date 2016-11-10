#ifndef _REC_EA09_MESSAGES_SETFPGAPOWER_H_
#define _REC_EA09_MESSAGES_SETFPGAPOWER_H_

#include "rec/ea09/messages/util.h"

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class SetFPGAPower
			{
			public:
				static void encode( unsigned char* buffer, bool setOn )
				{
					buffer[0] = 27;
					buffer[1] = 4;
					buffer[2] = 0;
					buffer[3] = ( setOn ? 1 : 0 );

					buffer[2] = generate_checkSum( buffer );
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_SETFPGAPOWER_H_
