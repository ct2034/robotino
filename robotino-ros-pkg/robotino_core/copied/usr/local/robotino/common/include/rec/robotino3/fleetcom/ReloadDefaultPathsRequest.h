#ifndef RELOADDEFAULTPATHSREQUEST_H
#define RELOADDEFAULTPATHSREQUEST_H

#include "rec/robotino3/fleetcom/Command.h"

namespace rec
{
    namespace robotino3
    {
        namespace fleetcom
        {
            class ReloadDefaultPathsRequest : public Request
            {
            public:
                ReloadDefaultPathsRequest()
                    : Request( Request::ReloadDefaultPathsRequestId )
                {
                }

                QString encode() const
                {
                    return "ReloadDefaultPaths";
                }

				QString print() const
				{
					return encode();
				}
            };

            typedef QSharedPointer< ReloadDefaultPathsRequest > ReloadDefaultPathsRequestPointer;

        }
    }
}

#endif // RELOADDEFAULTPATHSREQUEST_H
