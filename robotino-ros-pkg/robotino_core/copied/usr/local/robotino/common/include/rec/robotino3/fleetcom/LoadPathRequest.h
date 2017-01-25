#ifndef _REC_ROBOTINO3_FLEETCOM_LOADPATHREQUEST_H
#define _REC_ROBOTINO3_FLEETCOM_LOADPATHREQUEST_H

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
    namespace robotino3
    {
        namespace fleetcom
        {
            class LoadPathRequest : public Request
            {
            public:
                LoadPathRequest()
                    : Request( Request::LoadPathRequestId )
                {
                }

                void encode(QString* message) const //, QVariantMap* pairs) const
                {
                    *message = "LoadPath";
                }

                QString print() const
                {
                    QString msg;
                    encode( &msg );
                    return QString("%1 %2").arg(msg).arg(". Path not available");
                }
            };

            typedef QSharedPointer< LoadPathRequest > LoadPathRequestPointer;

        }
    }
}

#endif // _REC_ROBOTINO3_FLEETCOM_LOADPATHREQUEST_H
