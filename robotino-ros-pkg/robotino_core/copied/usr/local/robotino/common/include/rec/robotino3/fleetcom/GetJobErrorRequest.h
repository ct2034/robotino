#ifndef _REC_ROBOTINO3_FLEETCOM_GetJobErrorRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_GetJobErrorRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetJobErrorRequest : public Request
			{
			public:
				GetJobErrorRequest(int jobId_)
					: Request(Request::GetJobErrorRequestId)
					, jobId(jobId_)
				{
				}

				QString encode() const
				{
					return QString("GetJobError %1").arg(jobId);
				}

				QString print() const
				{
					return encode();
				}

				const int jobId;
			};

			typedef QSharedPointer< GetJobErrorRequest > GetJobErrorRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GetJobErrorRequest_H_
