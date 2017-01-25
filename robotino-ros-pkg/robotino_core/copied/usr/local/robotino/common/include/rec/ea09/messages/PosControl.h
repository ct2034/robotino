#ifndef _REC_EA09_MESSAGES_POSCONTROL_H_
#define _REC_EA09_MESSAGES_POSCONTROL_H_

#include "rec/ea09/messages/util.h"

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class PosControl
			{
			public:
				static void encode(
					unsigned char* buffer,
					bool enable,
					unsigned char speed,
					unsigned short imax )
				{
					buffer[0] = 29;
					buffer[1] = 16;
					buffer[2] = 0;
					buffer[3] = (enable?1:0);
					buffer[4] = speed;
					buffer[5] = ( imax >> 8 );
					buffer[6] = ( imax & 0xFF );
					buffer[7] = 0;
					buffer[8] = 0;
					buffer[9] = 0;
					buffer[10] = 0;
					buffer[11] = 0;
					buffer[12] = 0;
					buffer[13] = 0;
					buffer[14] = 0;
					buffer[15] = 0;

					buffer[2] = generate_checkSum( buffer );
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_POSCONTROL_H_
