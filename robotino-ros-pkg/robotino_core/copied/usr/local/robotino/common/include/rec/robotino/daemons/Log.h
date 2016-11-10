#ifndef _REC_ROBOTINO_DAEMONS_LOG_H_
#define _REC_ROBOTINO_DAEMONS_LOG_H_

#include <QtCore>

namespace rec
{
	namespace robotino
	{
		namespace daemons
		{
			class Log : public QObject
			{
				Q_OBJECT
			public:
				Log( const QString& logFileName, int verbosity, bool isDaemon );
				~Log();

				static Log* singleton()
				{
					Q_ASSERT( instance );
					return instance;
				}

				int verbosity() const { return _verbosity; }

			Q_SIGNALS:
				void logChanged( const QString&, int level );

			public Q_SLOTS:
				void log( const QString& message, int level );
				void log( const QStringList& message, int level );
				void setVerbosity( int level );

			private:
				void customEvent( QEvent* e );

				QString _logFileName;
				int _verbosity;
				bool _isDaemon;
				QString _lastLog;
				QMutex _mutex;

				static Log* instance;

				class LogEvent : public QEvent
				{
				public:
					LogEvent( const QString& message_, int level_ )
						: QEvent( QEvent::User )
						, message( message_ )
						, level( level_ )
					{
					}

					const QString message;
					const int level;
				};
			};
		}
	}
}

#endif //_REC_ROBOTINO_DAEMONS_LOG_H_
