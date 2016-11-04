#ifndef _REC_ROBOTINO3_GRIPPERCOM_COM_H_
#define _REC_ROBOTINO3_GRIPPERCOM_COM_H_

#include "rec/robotino3/serialio/Com.h"

namespace rec
{
	namespace robotino3
	{
		namespace grippercom
		{
			class Decoder;

			class Com : public rec::robotino3::serialio::Com
			{
			public:
				Com();

				void resetLpc( bool enterUSBBootloader );
				void getSensorReadings();
				void setPwm( int servo, int ratio );
				void setLed( int led, bool on );

				virtual void sensorReadingsCb( float i1, float i2, float ad3, float ad4, float ad6 );

			private:
				void parse_i( rec::robotino3::serialio::TagPointer p );
			};
		}
	}
}

#endif //_REC_ROBOTINO3_GRIPPERCOM_COM_H_
