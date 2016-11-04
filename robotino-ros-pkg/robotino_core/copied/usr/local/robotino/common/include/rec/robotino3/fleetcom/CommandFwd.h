#ifndef _REC_ROBOTINO3_FLEETCOM_COMMANDFWD_H_
#define _REC_ROBOTINO3_FLEETCOM_COMMANDFWD_H_

#include <QtCore>

namespace rec
{
	namespace robotino3
	{
		namespace fleetcom
		{
			class Command;
			typedef QSharedPointer< Command > CommandPointer;

			class Response;
			typedef QSharedPointer< Response > ResponsePointer;

			class Request;
			typedef QSharedPointer< Request > RequestPointer;
		}
	}
}

#endif //_REC_ROBOTINO3_FLEETCOM_COMMANDFWD_H_
