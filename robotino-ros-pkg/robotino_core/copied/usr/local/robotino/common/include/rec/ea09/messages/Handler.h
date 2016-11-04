#ifndef _REC_EA09_MESSAGES_HANDLER_H_
#define _REC_EA09_MESSAGES_HANDLER_H_

namespace rec
{
	namespace ea09
	{
		namespace messages
		{
			class Handler
			{
			public:
				Handler();
				
				~Handler();

				/**
				Process the given message. If message if correct the corresponding _Received function is called
				@return Returns false if message is corrupt. Returns true otherwise.
				@see ipAddressReceived
				*/
				bool processMessage( const unsigned char* message );

				virtual void informationReceived( unsigned char major, unsigned char minor, unsigned char patch, bool isEthernetAvailable );

				virtual void ipAddressReceived(
					unsigned char ip1, unsigned char ip2, unsigned char ip3, unsigned char ip4,
					unsigned char mask1, unsigned char mask2, unsigned char mask3, unsigned char mask4 );

				virtual void fpgaPowerReceived( bool isPowerOn );

				virtual void errorReceived( unsigned char messageId );

				virtual void posControlReceived( bool enabled, unsigned short speed, unsigned short imax );
			};
		}
	}
}

#endif //_REC_EA09_MESSAGES_HANDLER_H_
