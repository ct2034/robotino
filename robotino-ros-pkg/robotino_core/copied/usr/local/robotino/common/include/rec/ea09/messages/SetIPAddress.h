#ifndef _REC_EA09_MESSAGES_SETIPADDRESS_H_
#define _REC_EA09_MESSAGES_SETIPADDRESS_H_

#include "rec/ea09/messages/util.h"

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class SetIPAddress
			{
			public:
				static void encode(
					unsigned char* buffer,
					unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4,
					unsigned char mask1, unsigned char mask2, unsigned char mask3, unsigned char mask4 )
				{
					buffer[0] = 22;
					buffer[1] = 11;
					buffer[2] = 0;
					buffer[3] = ip1;
					buffer[4] = ip2;
					buffer[5] = ip3;
					buffer[6] = ip4;
					buffer[7] = mask1;
					buffer[8] = mask2;
					buffer[9] = mask3;
					buffer[10] = mask4;

					buffer[2] = generate_checkSum( buffer );
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_GETIPADDRESS_H_
