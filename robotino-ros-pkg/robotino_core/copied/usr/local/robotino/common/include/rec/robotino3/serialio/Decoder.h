#ifndef _REC_ROBOTINO3_SERIALIO_DECODER_H_
#define _REC_ROBOTINO3_SERIALIO_DECODER_H_

#include "rec/robotino3/serialio/TagFwd.h"

namespace rec
{
	namespace robotino3
	{
		namespace serialio
		{
			class Decoder
			{
			public:
				Decoder()
				{
				}

				virtual ~Decoder()
				{
				}

				/**
				* @throws TagExcpetion
				*/
				rec::robotino3::serialio::TagList decode( const QByteArray& data );

				/**
				* @throws TagExcpetion
				*/
				rec::robotino3::serialio::TagList decode( QIODevice* buffer );

			protected:
				virtual rec::robotino3::serialio::TagPointer decode( int value, QIODevice* buffer ) = 0;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_DECODER_H_

