//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"
#include "MyCamera.h"

#include "CameraView.h"
#include "AddressInput.h"
#include "CameraSettings.h"

MainWindow::MainWindow()
	: _camera( new MyCamera )
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	CameraView* cameraView = new CameraView( this, _camera );
	CameraSettings* settings = new CameraSettings( this, _camera );

	QDockWidget* settingsDock = new QDockWidget( "Camera settings", this );
	settingsDock->setWidget( settings );
	settingsDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::LeftDockWidgetArea, settingsDock );
	settingsDock->setFloating( false );
	settingsDock->setVisible( true );

	setCentralWidget( cameraView );
}
