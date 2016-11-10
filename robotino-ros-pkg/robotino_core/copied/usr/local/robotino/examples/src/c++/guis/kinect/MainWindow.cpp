//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"
#include "MyKinect.h"
#include "Settings.h"
#include "TabView.h"

#include "AddressInput.h"

MainWindow::MainWindow()
	: _kinect( new MyKinect )
	, _tabView( new TabView( this, _kinect ) )
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	Settings* settings = new Settings( this, _kinect );

	QDockWidget* settingsDock = new QDockWidget( "Settings", this );
	settingsDock->setWidget( settings );
	settingsDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::LeftDockWidgetArea, settingsDock );
	settingsDock->setFloating( false );
	settingsDock->setVisible( true );

	setCentralWidget( _tabView );
}
