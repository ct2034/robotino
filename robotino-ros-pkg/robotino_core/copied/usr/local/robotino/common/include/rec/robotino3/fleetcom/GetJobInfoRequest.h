#ifndef _REC_ROBOTINO3_FLEETCOM_GetJobInfoRequest_H_
#define _REC_ROBOTINO3_FLEETCOM_GetJobInfoRequest_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class GetJobInfoRequest : public Request
			{
			public:
				GetJobInfoRequest(int jobId_)
					: Request( Request::GetJobInfoRequestId )
					, jobId(jobId_)
				{
				}

				QString encode() const
				{
					return QString("GetJobInfo %1").arg(jobId);
				}

				QString print() const
				{
					return encode();
				}

				const int jobId;
			};

			typedef QSharedPointer< GetJobInfoRequest > GetJobInfoRequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_GetJobInfoRequest_H_
