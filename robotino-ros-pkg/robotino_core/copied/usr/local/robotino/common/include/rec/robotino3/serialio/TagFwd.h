#ifndef _REC_ROBOTINO3_SERIALIO_TAG_TAGFWD_H_
#define _REC_ROBOTINO3_SERIALIO_TAG_TAGFWD_H_

#include <QtCore>

namespace rec
{
	namespace robotino3
	{
		namespace serialio
		{
			class Tag;
			typedef QSharedPointer< Tag > TagPointer;
			typedef QList< TagPointer > TagList;
			typedef QMap< QString, TagPointer > TagMap;

			class TagHelper
			{
			public:
				TagHelper( rec::robotino3::serialio::TagPointer p )
					: _p( p )
				{
				}

				rec::robotino3::serialio::TagPointer _p;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_TAG_TAGFWD_H_
