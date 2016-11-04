#include "LogWindow.h"
#include "LogView.h"
#include "Log.h"

LogWindow::LogWindow( QAction* hideShowAction, QWidget* parent )
: QDockWidget( parent )
, _hideShowAction( hideShowAction )
{
	setObjectName( "LogMessageWindow" );
	setWindowTitle( tr("Log") );

	LogView* v = new LogView;
	v->setDocument( &recQtLog );

	setWidget( v );

	if( _hideShowAction )
	{
		connect( _hideShowAction, SIGNAL( triggered() ), SLOT( on_logmessage_triggered() ) );
	}
}

void LogWindow::updateAction()
{
	if( _hideShowAction )
	{
		_hideShowAction->setChecked( isVisible() );
	}
}

void LogWindow::showEvent( QShowEvent* e )
{
	if( _hideShowAction )
	{
		_hideShowAction->setChecked( true );
	}
	e->accept();
}

void LogWindow::hideEvent( QHideEvent* e )
{
	if( _hideShowAction )
	{
		_hideShowAction->setChecked( false );
	}
	e->accept();
}

void LogWindow::on_logmessage_triggered()
{
	setVisible( _hideShowAction->isChecked() );
}

