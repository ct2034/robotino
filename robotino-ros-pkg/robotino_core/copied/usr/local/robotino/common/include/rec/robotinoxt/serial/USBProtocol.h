#ifndef _USBPROTOCOL_H_
#define _USBPROTOCOL_H_

/*PC -> LPC*/
#define REC_ROBOTINOXT_ResetDevice				0
#define REC_ROBOTINOXT_GetInfo					1
#define REC_ROBOTINOXT_SetPressures				2
#define REC_ROBOTINOXT_SetDOut					3
#define REC_ROBOTINOXT_SetCompressorsEnabled	4
#define REC_ROBOTINOXT_NUMMESSAGES_PC2LPC		5

/*LPC -> PC*/
#define REC_ROBOTINOXT_Error					0
#define REC_ROBOTINOXT_Info						1
#define REC_ROBOTINOXT_Status					2

/*Status error codes*/
#define REC_ROBOTINOXT_STATUS_OK					0
#define REC_ROBOTINOXT_STATUS_ErrorReadPressures	1
#define REC_ROBOTINOXT_STATUS_ErrorReadIOs			2

/*Error codes*/
#define REC_ROBOTINOXT_InvalidMessageLength		0
#define REC_ROBOTINOXT_InvalidMessageType		1
#define REC_ROBOTINOXT_KeepAlive				0xFF

static unsigned char rec_robotinoxt_lpcToPC_checksum( const char* message )
{
	unsigned char i;
	unsigned short sum = message[0];
	sum += message[1];

	for( i=3; i<message[0]; i++ )
	{
		sum += message[i];
	}

	return ~( sum & 0xFF );
}

static unsigned char rec_robotinoxt_isMessageCorrect( const char* message )
{
	unsigned char i;
	unsigned short sum = 0;

	for( i=0; i<message[0]; i++ )
	{
		sum += message[i];
	}

	sum = sum & 0xFF;

	return ( sum == 0xFF );
}


#ifdef HAVE_QT
#include <QtCore>

namespace rec
{
	namespace robotinoxt
	{
		namespace serial
		{
			class USBProtocol
			{
			public:
				static bool isMessageValid( const char* data, unsigned int dataSize );

				static bool isKeepAliveMessage( const char* data, unsigned int dataSize );

				static bool decodeInfoMessage( const char* data, unsigned int dataSize, int* majorVer, int* minorVer, int* patchVer );

				static QByteArray encodeSetPressuresMessage( const QVector< qint16 >& pressures );

				static bool decodeStatusMessage( const char* data, unsigned int dataSize, QVector< qint16 >* pressures, bool* pressureSensor, QVector< qint16 >* potis );

				static QByteArray encodeSetDOutMessage( unsigned int number, bool enabled );

				static const unsigned char getInfoMessage[];

				static const unsigned char resetDeviceMessage[];
				static const unsigned char resetDeviceEnterBootloaderMessage[];

				static const unsigned char setCompressorsEnabledMessage[];
				static const unsigned char setCompressorsDisabledMessage[];
			};
		}
	}
}
#endif //HAVE_QT

#endif //_USBPROTOCOL_H_
