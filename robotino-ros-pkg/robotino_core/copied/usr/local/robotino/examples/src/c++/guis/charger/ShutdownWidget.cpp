#include "ShutdownWidget.h"

ShutdownWidget::ShutdownWidget()
{
	setMinimumSize( 100, 40 );
	setFloatable( false );
	setMovable( false );

	QPushButton* shutdownButton = new QPushButton( "Shutdown", this );
	bool ok = connect( shutdownButton, SIGNAL( clicked() ), SLOT( on_shutdown_clicked() ) );
	Q_ASSERT( ok );

	addWidget( shutdownButton );
}

void ShutdownWidget::on_shutdown_clicked()
{
	this->shutdown();
}

