#ifndef _REC_QT_LOG_LOG_H_
#define _REC_QT_LOG_LOG_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


#define recQtLog Log::singleton()

class Log : public QTextDocument
{
	Q_OBJECT
public:
	/// Singleton methods
	static Log& singleton(void);
	static void init(void);
	static void done(void);

	~Log();

	void postLog (const QString& message, int verbosity=0xFF);
	void log( const QString& message );
	void log( const QStringList& messages );

	void postEmptyLine();
	void insertEmptyLine();

private:
	Log();

	void customEvent( QEvent * event );

	static Log* _impl;

public:
	enum{ MessageReceivedEventType = QEvent::User };

	class MessageReceivedEvent : public QEvent
	{
	public:
		MessageReceivedEvent (int verbosity, const QString& message)
			: QEvent((QEvent::Type)MessageReceivedEventType)
			, _verbosity(verbosity)
			, _message(message)
		{
		}

		const int& verbosity (void) {
			return _verbosity;
		}

		const QString& message (void) {
			return _message;
		}
	private:
		const int _verbosity;
		const QString _message;
	};
};

#endif //_REC_QT_LOG_LOG_H_

