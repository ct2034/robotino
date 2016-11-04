//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "FormatList.h"
#include "MyCamera.h"

FormatList::FormatList( QWidget* parent, MyCamera* camera )
	: QListWidget( parent )
	, _camera( camera )
{
	bool ok = connect( this, SIGNAL( itemClicked ( QListWidgetItem* ) ), SLOT( on_itemClicked ( QListWidgetItem* ) ) );
	Q_ASSERT( ok );

	ok = connect( _camera, SIGNAL( capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& ) ), SLOT( on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& ) ) );
	Q_ASSERT( ok );

	ok = connect( _camera, SIGNAL( settingsChanged( unsigned int, unsigned int, const QString& ) ), SLOT( on_camera_settingsChanged( unsigned int, unsigned int, const QString& ) ) );
	Q_ASSERT( ok );
}

void FormatList::on_itemClicked ( QListWidgetItem* item )
{
	QStringList l = item->text().split( " " );
	QStringList sizeList = l.at(0).split( "x" );

	_camera->setFormat( sizeList.at(0).toUInt(), sizeList.at(1).toUInt(), l.at( 1 ).toLatin1().constData() );
}

void FormatList::on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& capabilities )
{
	clear();

	for( unsigned int i=0; i<capabilities.numFormats(); ++i )
	{
		rec::robotino::api2::CameraFormat f = capabilities.format( i );
		QString format = f.name();
		addItem( QString( "%1x%2 %3" ).arg( f.width ).arg( f.height ).arg( format ) );
	}
}

void FormatList::on_camera_settingsChanged( unsigned int width, unsigned int height, const QString& format )
{
	QString label = QString( "%1x%2 %3" ).arg( width ).arg( height ).arg( format );

	for( int i=0; i<count(); ++i )
	{
		QListWidgetItem* it = item( i );
		it->setIcon( QIcon() );
	}

	QList<QListWidgetItem*> l =	findItems( label, Qt::MatchExactly );

	if( 1 == l.size() )
	{
		l.at(0)->setIcon( QPixmap( ":/icons/green_dot.png" ) );
	}
}
