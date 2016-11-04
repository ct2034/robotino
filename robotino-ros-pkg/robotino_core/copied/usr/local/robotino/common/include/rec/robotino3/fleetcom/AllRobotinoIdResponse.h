#ifndef _REC_ROBOTINO3_FLEETCOM_ALLROBOTINOIDRESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_ALLROBOTINOIDRESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class AllRobotinoIdResponse : public Response
			{
			public:
				AllRobotinoIdResponse( const QString& message, unsigned int sequence );

				QVector<int> allRobotinoIds() const { return _allRobotinoIds; }

				QString print() const;

			private:
				QVector<int> _allRobotinoIds;
			};

			typedef QSharedPointer< AllRobotinoIdResponse > AllRobotinoIdResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ALLROBOTINOIDRESPONSE_H_