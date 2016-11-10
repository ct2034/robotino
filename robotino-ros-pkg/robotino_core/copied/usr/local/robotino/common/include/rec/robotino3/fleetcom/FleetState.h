#ifndef _REC_ROBOTINO3_FLEETCOM_FLEETSTATE_H_
#define _REC_ROBOTINO3_FLEETCOM_FLEETSTATE_H_

#include <QtCore>

#include "rec/robotino3/fleetcom/RobotinoState.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class FleetState
			{
			public:
				FleetState()
				{
				}

				void clear()
				{
					states.clear();
				}

				QVector<RobotinoState> states;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_FLEETSTATE_H_
