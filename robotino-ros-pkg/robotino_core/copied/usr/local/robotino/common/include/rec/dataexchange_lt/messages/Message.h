#ifndef _REC_DATAEXCHANGE_MESSAGES_MESSAGE_H_
#define _REC_DATAEXCHANGE_MESSAGES_MESSAGE_H_

#include "rec/dataexchange_lt/defines.h"

namespace rec
{
	namespace dataexchange_lt
	{
		namespace messages
		{
			typedef enum
			{
				ConfigurationId = 0,
				DataId,
				RegisterChannelId,
				UnregisterChannelId
			} Id;
		}
	}
}

#endif //_REC_DATAEXCHANGE_MESSAGES_MESSAGE_H_
