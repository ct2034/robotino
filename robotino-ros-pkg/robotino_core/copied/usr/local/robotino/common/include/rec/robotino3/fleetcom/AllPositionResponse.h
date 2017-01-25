#ifndef _REC_ROBOTINO3_FLEETCOM_ALLPOSITIONRESPONSE_H_
#define _REC_ROBOTINO3_FLEETCOM_ALLPOSITIONRESPONSE_H_

#include "rec/robotino3/fleetcom/Command.h"
#include "rec/robotino3/fleetcom/Position.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class AllPositionResponse : public Response
			{
			public:
				AllPositionResponse( const QString& message, unsigned int sequence );

				PositionList positions() const { return _positions; }

				QString print() const;

			private:
				PositionList _positions;
			};

			typedef QSharedPointer< AllPositionResponse > AllPositionResponsePointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_ALLPOSITIONRESPONSE_H_