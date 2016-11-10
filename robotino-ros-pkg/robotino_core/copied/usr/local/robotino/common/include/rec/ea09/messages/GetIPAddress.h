#ifndef _REC_EA09_MESSAGES_GETIPADDRESS_H_
#define _REC_EA09_MESSAGES_GETIPADDRESS_H_

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class GetIPAddress
			{
			public:
				static void encode( unsigned char* buffer )
				{
					buffer[0] = 21;
					buffer[1] = 3;
					buffer[2] = 0xE7;
				}
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_GETIPADDRESS_H_
