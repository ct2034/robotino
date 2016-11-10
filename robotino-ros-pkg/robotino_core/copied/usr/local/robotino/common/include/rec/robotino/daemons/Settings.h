#ifndef _REC_ROBOTINO_DAEMONS_SETTINGS_H_
#define _REC_ROBOTINO_DAEMONS_SETTINGS_H_

#include <QtCore>

namespace rec
{
	namespace robotino
	{
		namespace daemons
		{
			class Settings : public QObject
			{
				Q_OBJECT
			public:
				static Settings* singleton()
				{
					Q_ASSERT( _impl );
					return _impl;
				}

				static void init();

				static void done();

				void read( const QString& path, const QString& userpath );

				QVariant value( const QString& key, const QVariant& defaultValue = QVariant() );

				void setValue( const QString& key, const QVariant& value );

				QMap< QString, QVariant > parameters() const;

			Q_SIGNALS:
				void log( const QString& message, int level );

			private:
				Settings();
				~Settings();

				static Settings* _impl;

				QSettings* _settings;
				QSettings* _user_settings;
				QString _user_settings_path;
			};
		}
	}
}

#endif //_REC_ROBOTINO_DAEMONS_SETTINGS_H_
