#ifndef _REC_ROBOTINO3_FLEETCOM_PUSHJOBREQUEST_H_
#define _REC_ROBOTINO3_FLEETCOM_PUSHJOBREQUEST_H_

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class PushJobRequest : public Request
			{
			public:
				PushJobRequest(const QString& jobType_, const int jobId_, const int priority_, const int robotinoId_, const int fromStation_, const int fromBelt_, const bool isFromManual_, const int toStation_, const int toBelt_, const bool isToManual_)
					: Request( Request::PushJobRequestId )
					, jobType(jobType_)
					, jobId(jobId_)
					, priority(priority_)
					, robotinoId(robotinoId_)
					, fromStation(fromStation_)
					, fromBelt(fromBelt_)
					, isFromManual(isFromManual_)
					, toStation(toStation_)
					, toBelt(toBelt_)
					, isToManual(isToManual_)
				{
				}

				QString encode() const
				{
					/*PushJob JOBTYPE JOBID PRIORITY ROBOTINOID FROMSTATION FROMBELT IsFromManual TOSTATION TOBELT IsToManual*/
					return QString("PushJob %1 %2 %3 %4 %5 %6 %7 %8 %9 %10").arg(jobType).arg(jobId).arg(priority).arg(robotinoId).arg(fromStation).arg(fromBelt).arg(isFromManual ? "true" : "false").arg(toStation).arg(toBelt).arg(isToManual ? "true" : "false");
				}

				QString print() const
				{
					return encode();
				}

				const QString jobType;
				const int jobId;
				const int priority;
				const int robotinoId;
				const int fromStation;
				const int fromBelt;
				const bool isFromManual;
				const int toStation;
				const int toBelt;
				const bool isToManual;
			};

			typedef QSharedPointer< PushJobRequest > PushJobRequestPointer;

		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_PUSHJOBREQUEST_H_
