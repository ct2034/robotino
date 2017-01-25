#ifndef _REC_ROBOTINO3_SERIALIO_TAG_H_
#define _REC_ROBOTINO3_SERIALIO_TAG_H_

#include "rec/robotino3/serialio/TagFwd.h"

namespace rec
{
	namespace robotino3
	{
		namespace serialio
		{
			typedef enum {
				TAG_GET_HW_VERSION						=	1,
				TAG_HW_VERSION							=	2,
				TAG_GET_SW_VERSION						=	3,
				TAG_SW_VERSION							=	4
			} Tag_Value_t;

			class Tag
			{
			public:
				virtual ~Tag()
				{
				}

				virtual bool isRoot() const { return false; }

				int value() const { return _value; }

				virtual QByteArray encode() const
				{
					Q_ASSERT( false );
					return QByteArray();
				}

				virtual QString signature() const { return QString( "%1" ).arg( _value ); };

				virtual QString print() const { return QString( "not implemented %1" ).arg( _value ); };

			protected:
				Tag( int value )
					: _value( value )
				{
				}

			private:
				int _value;
			};

			class TagException : public std::exception
			{
			public:
				TagException( const QString& message )
					: msg( message.toStdString() )
				{
				}

				virtual ~TagException() throw ()
				{
				}

				virtual const std::string& getMessage() const
				{
					return msg;
				}

				//Compatibility functions for std::exception
				virtual const char* what() const throw ()
				{
					return msg.c_str();
				}

			protected:
				std::string msg;
			};
		}
	}
}

#endif //_REC_ROBOTINO3_SERIALIO_TAG_TAG_H_
