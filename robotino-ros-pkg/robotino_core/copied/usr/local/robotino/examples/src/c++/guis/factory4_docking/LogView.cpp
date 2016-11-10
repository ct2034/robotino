#include "LogView.h"

LogView::LogView( QWidget* parent )
: QTextEdit( parent )
{
	setReadOnly( true );

	connect( this, SIGNAL( textChanged() ), SLOT( on_textChanged() ) );
}


void LogView::on_textChanged()
{
	ensureCursorVisible();
}
