#include "Log.h"

Log* Log::_impl = NULL;

Log::Log()
{
  setMaximumBlockCount( 100 );
}

Log::~Log()
{
}

Log& Log::singleton(void)
{
	return *_impl;
}

void Log::init(void)
{
	Q_ASSERT( NULL == _impl );
	_impl = new Log;
}

void Log::done(void)
{
	delete _impl;
	_impl = NULL;
}

void Log::postEmptyLine()
{
	qApp->postEvent( this, new MessageReceivedEvent(0xFF, QString::null));
}

void Log::insertEmptyLine()
{
	log( QString::null );
}

void Log::postLog (const QString& message, int verbosity) {
	qApp->postEvent(this, new MessageReceivedEvent(verbosity, message));
}

void Log::log( const QString& message )
{
	QTextCursor cursor( this );
	cursor.movePosition( QTextCursor::End );

	cursor.insertBlock();
	cursor.insertText( message );
}

void Log::log( const QStringList& messages )
{
	Q_FOREACH( const QString& str, messages )
	{
		log( str );
	}
}

void Log::customEvent (QEvent* event) {
	switch (event->type()) {
		case MessageReceivedEventType: {
			MessageReceivedEvent* e = static_cast<MessageReceivedEvent*>(event);
			log(e->message());
		}	break;
		default:
			break;
	}
}
