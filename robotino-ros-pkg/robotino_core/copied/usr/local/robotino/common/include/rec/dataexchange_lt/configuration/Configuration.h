#ifndef _REC_DATAEXCHANGE_CONFIGURATION_CONFIGURATION_H_
#define _REC_DATAEXCHANGE_CONFIGURATION_CONFIGURATION_H_

#include "rec/dataexchange_lt/defines.h"
#include "rec/dataexchange_lt/types.h"
#include "rec/dataexchange_lt/messages/Data.h"

#include <QObject>
#include <QByteArray>
#include <QMap>
#include <QDomDocument>
#include <QDomElement>
#include <QMutex>

namespace rec
{
	namespace dataexchange_lt
	{
		namespace configuration
		{
			class ConfigurationLocker;

			class REC_DATAEXCHANGE_EXPORT Configuration : public QObject
			{
				Q_OBJECT
				friend class ConfigurationLocker;
			public:
				Configuration( QObject* parent = NULL );

				Configuration( const Configuration& other );

				Configuration& operator=( const Configuration& other );

				bool isEmpty() const { return _itemFromName.isEmpty(); }

				bool contains( const QString& name ) const { return _itemFromName.contains( name ); }

				QMap< QString, rec::dataexchange_lt::messages::Data > itemContainer() const { return _itemFromName; }

				rec::dataexchange_lt::messages::Data data( const QString& name ) const;
				bool setData( const rec::dataexchange_lt::messages::Data& data );

				/**
				@return Returns false if the name is already in use of if name or type are empty strings.
				*/
				bool addItem( const QString& name, rec::dataexchange_lt::DataType type );

				bool addItem( const rec::dataexchange_lt::messages::Data& item );

				bool removeItem( const QString& name );

				bool renameItem( const QString& oldName, const QString& newName );

				bool changeType( const QString& name, rec::dataexchange_lt::DataType newType );

				rec::dataexchange_lt::DataType type( const QString& name ) const;

				QByteArray save() const;

				bool load( const QByteArray& data );

			Q_SIGNALS:
				void changed();

			private:
				bool addItem_i( const QString& name, rec::dataexchange_lt::DataType type );
				
				QMap< QString, rec::dataexchange_lt::messages::Data > _itemFromName;
				mutable QMutex _mutex;
			};

			class ConfigurationLocker
			{
			public:
				ConfigurationLocker( Configuration& configuration_ )
					: configuration( configuration_ )
				{
					configuration._mutex.lock();
				}

				~ConfigurationLocker()
				{
					configuration._mutex.unlock();
				}

				Configuration& configuration;
			};
		}
	}
}

#include <QMetaType>
Q_DECLARE_METATYPE( rec::dataexchange_lt::configuration::Configuration )

#endif //_REC_DATAEXCHANGE_CONFIGURATION_CONFIGURATION_H_
