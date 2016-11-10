#ifndef _REC_ROBOTINO3_FLEETCOM_SAVEPATHREQUEST_H
#define _REC_ROBOTINO3_FLEETCOM_SAVEPATHREQUEST_H

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
    namespace robotino3
    {
        namespace fleetcom
        {
            class SavePathRequest : public Request
            {
            public:
                SavePathRequest( const QByteArray& data )
                    : Request( Request::SavePathRequestId )
                    , _data(data)
                {
                }

                SavePathRequest(const QVariantMap& pairs)
                    : Request(Request::SavePathRequestId)
                {
                    _data = pairs["data"].toByteArray();
                }

                void encode(QString* message, QVariantMap* pairs) const
                {
                    *message = "SavePath";
                    (*pairs)["data"] = _data;
                }

                QByteArray data() const { return _data; }

                QString print() const
                {
                    QString msg;
                    QVariantMap pairs;
                    encode( &msg, &pairs );
                    return QString("%1 size:%2bytes").arg(msg).arg(data().size());
                }

            private:
                QByteArray _data;
            };

            typedef QSharedPointer< SavePathRequest > SavePathRequestPointer;

        }
    }
}


#endif // _REC_ROBOTINO3_FLEETCOM_SAVEPATHREQUEST_H
