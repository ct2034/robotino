#ifndef _REC_ROBOTINO3_FLEETCOM_ROBOTINOSTATE_H_
#define _REC_ROBOTINO3_FLEETCOM_ROBOTINOSTATE_H_

#include <QString>

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class RobotinoState
			{
			public:
				RobotinoState()
					: id( invalidId )
					, x(0.0)
					, y(0.0)
					, phi(0.0)
					, batteryVoltage(0.0)
				{
				}

				int id;
				float x;
				float y;
				float phi;
				float batteryVoltage;
				QString ipAddress;
				
				static const int invalidId = -1;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ROBOTINOSTATE_H_
