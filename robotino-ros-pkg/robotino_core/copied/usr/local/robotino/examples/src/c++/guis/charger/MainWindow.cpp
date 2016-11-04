//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "MainWindow.h"

#include "MyCom.h"

#include "AddressInput.h"
#include "ChargerWidget.h"
#include "PowerManagementWidget.h"
#include "ShutdownWidget.h"

MainWindow::MainWindow()
{
	AddressInput* addressInput = new AddressInput( this );
	this->addToolBar( addressInput );

	ShutdownWidget* shutdownWidget = new ShutdownWidget;
	this->addToolBar( Qt::BottomToolBarArea, shutdownWidget );

	for( int i=0; i<3; ++i )
	{
		QDockWidget* chargerDock = new QDockWidget( QString("Charger %1").arg(i), this );
		chargerDock->setWidget( new ChargerWidget( i ) );
		chargerDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
		addDockWidget(Qt::BottomDockWidgetArea, chargerDock );
		chargerDock->setFloating( false );
		chargerDock->setVisible( true );
	}

	QDockWidget* powerManagementDock = new QDockWidget( QString("Power management"), this );
	powerManagementDock->setWidget( new PowerManagementWidget );
	powerManagementDock->setFeatures( QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable );
	addDockWidget(Qt::BottomDockWidgetArea, powerManagementDock );
	powerManagementDock->setFloating( false );
	powerManagementDock->setVisible( true );
}
