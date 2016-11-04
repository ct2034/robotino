//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"

#include "AddressInput.h"
#include "DigitalInputWidget.h"
#include "DigitalOutputWidget.h"
#include "RelayWidget.h"
#include "BumperWidget.h"

MainWindow::MainWindow()
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	QDockWidget* digitalInputDock = new QDockWidget( "Digital input", this );
	digitalInputDock->setWidget( new DigitalInputWidget );
	digitalInputDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::TopDockWidgetArea, digitalInputDock );
	digitalInputDock->setFloating( false );
	digitalInputDock->setVisible( true );

	QDockWidget* digitaloutputDock = new QDockWidget( "Digital output", this );
	digitaloutputDock->setWidget( new DigitalOutputWidget );
	digitaloutputDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::TopDockWidgetArea, digitaloutputDock );
	digitaloutputDock->setFloating( false );
	digitaloutputDock->setVisible( true );

	QDockWidget* relayDock = new QDockWidget( "Relays", this );
	relayDock->setWidget( new RelayWidget );
	relayDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, relayDock );
	relayDock->setFloating( false );
	relayDock->setVisible( true );

	QDockWidget* bumperDock = new QDockWidget( "Bumper", this );
	bumperDock->setWidget( new BumperWidget );
	bumperDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, bumperDock );
	bumperDock->setFloating( false );
	bumperDock->setVisible( true );

	//setCentralWidget( new ControllerWidget( this, _com ) );
}
