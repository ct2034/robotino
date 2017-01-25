//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"

#include "AddressInput.h"
#include "ControlPanelWidget.h"
#include "MotorWidget.h"
#include "OdometryWidget.h"

MainWindow::MainWindow()
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	QDockWidget* motor0Dock = new QDockWidget( "Motor 0", this );
	motor0Dock->setWidget( new MotorWidget( this, 0 ) );
	motor0Dock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, motor0Dock );
	motor0Dock->setFloating( false );
	motor0Dock->setVisible( true );

	QDockWidget* motor1Dock = new QDockWidget( "Motor 1", this );
	motor1Dock->setWidget( new MotorWidget( this, 1 ) );
	motor1Dock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, motor1Dock );
	motor1Dock->setFloating( false );
	motor1Dock->setVisible( true );

	QDockWidget* motor2Dock = new QDockWidget( "Motor 2", this );
	motor2Dock->setWidget( new MotorWidget( this, 2 ) );
	motor2Dock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, motor2Dock );
	motor2Dock->setFloating( false );
	motor2Dock->setVisible( true );

	QDockWidget* odometryDock = new QDockWidget( "Odometry", this );
	odometryDock->setWidget( new OdometryWidget( this ) );
	odometryDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, odometryDock );
	odometryDock->setFloating( false );
	odometryDock->setVisible( true );

	setCentralWidget( new ControlPanelWidget( this ) );
}
