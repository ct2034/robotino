//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"
#include "ControlBar.h"
#include "View.h"
#include "Scene.h"
#include "LogWindow.h"
#include "StationDetector.h"
#include "Driver.h"
#include "Transform.h"

#include "AddressInput.h"

MainWindow::MainWindow()
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	ControlBar* controlBar = new ControlBar( this );
	this->addToolBar( controlBar );

	LogWindow* logWindow = new LogWindow( NULL, this );
	logWindow->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, logWindow );
	logWindow->setFloating( false );
	logWindow->setVisible( true );

	_transform = new Transform;

	Scene* scene = new Scene( _transform, this );
	
	View* view = new View( this );
	view->setScene( scene );

	_stationDetector = new StationDetector(_transform );

	bool ok;

	ok = connect( _stationDetector, SIGNAL( markersAt( const QVector< Marker >& ) ), scene, SLOT( setMarkers( const QVector< Marker >& ) ) );
	Q_ASSERT( ok );

	ok = connect( _stationDetector, SIGNAL( stationsAt( const QVector< Station >& ) ), scene, SLOT( setStations( const QVector< Station >& ) ) );
	Q_ASSERT( ok );

	setCentralWidget( view );

	_driver = new Driver(_transform, _stationDetector );

	ok = connect( controlBar, SIGNAL( start( int ) ), _driver, SLOT( start( int ) ) );
	Q_ASSERT( ok );

	ok = connect( controlBar, SIGNAL( stop() ), _driver, SLOT( stop() ) );
	Q_ASSERT( ok );
}
