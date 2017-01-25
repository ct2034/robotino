#ifndef _REC_ROBOTINO3_IOCOM_DECODER_H_
#define _REC_ROBOTINO3_IOCOM_DECODER_H_

#include "rec/robotino3/serialio/Decoder.h"

namespace rec
{
	namespace robotino3
	{
		namespace iocom
		{
			class Decoder : public rec::robotino3::serialio::Decoder
			{
			public:
				Decoder()
				{
				}

				virtual ~Decoder()
				{
				}

			protected:
				rec::robotino3::serialio::TagPointer decode( int value, QIODevice* buffer );
			};
		}
	}
}

#endif //_REC_ROBOTINO3_IOCOM_DECODER_H_

