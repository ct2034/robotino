#ifndef _REC_ROBOTINO3_FLEETCOM_LOADPATHRESPONSE_H
#define _REC_ROBOTINO3_FLEETCOM_LOADPATHRESPONSE_H

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
    namespace robotino3
    {
        namespace fleetcom
        {
            class LoadPathResponse : public Response
            {
            public:
                LoadPathResponse(const QString& message, const QVariantMap& pairs, unsigned int sequence);
                LoadPathResponse(const QByteArray& data);

                QByteArray data() const { return _data; }

                void encode(QString* message, QVariantMap* pairs) const
                {
                    *message = "LoadPathResponse";
                    (*pairs)["data"] = _data;
                }

                QString print() const;

            private:
                QByteArray _data;
            };

            typedef QSharedPointer< LoadPathResponse > LoadPathResponsePointer;
        }
    }
}

#endif // _REC_ROBOTINO3_FLEETCOM_LOADPATHRESPONSE_H
