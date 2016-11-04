#include "PowerManagementWidget.h"

PowerManagementWidget::PowerManagementWidget()
	: _battery_voltageLabel( new QLabel( this ) )
	, _system_currentLabel( new QLabel( this ) )
	, _ext_powerLabel( new QLabel( this ) )
	, _num_chargersLabel( new QLabel( this ) )
	, _batteryTypeLabel( new QLabel( this ) )
	, _batteryLowLabel( new QLabel( this ) )
	, _batteryLowShutdownCounterLabel( new QLabel( this ) )
{
	QFormLayout* layout = new QFormLayout;
	setLayout( layout );

	layout->addRow( "Battery voltage", _battery_voltageLabel );
	layout->addRow( "System current", _system_currentLabel );
	layout->addRow( "External power", _ext_powerLabel );
	layout->addRow( "Number of chargers", _num_chargersLabel );
	layout->addRow( "Battery type", _batteryTypeLabel );
	layout->addRow( "Battery low", _batteryLowLabel );
	layout->addRow( "Battery low shutdown in", _batteryLowShutdownCounterLabel );
}

void PowerManagementWidget::readingsEvent( float battery_voltage, float system_current, bool ext_power, int num_chargers, const char* batteryType, bool batteryLow, int batteryLowShutdownCounter )
{
	_battery_voltageLabel->setText( QString("%1V").arg( battery_voltage ) );
	_system_currentLabel->setText( QString("%1A").arg( system_current ) );
	_ext_powerLabel->setText( QString("%1").arg( ext_power ? "Yes" : "No" ) );
	_num_chargersLabel->setText( QString("%1").arg( num_chargers ) );
	_batteryTypeLabel->setText( QString("%1").arg( batteryType ) );
	_batteryLowLabel->setText( QString("%1").arg( batteryLow ? "Yes" : "No" ) );
	_batteryLowShutdownCounterLabel->setText( QString("%1").arg( batteryLowShutdownCounter ) );
}

