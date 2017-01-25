#include "ChargerWidget.h"

ChargerWidget::ChargerWidget( unsigned int number )
	: _timeLabel( new QLabel( this ) )
	, _batteryVoltageLabel( new QLabel( this ) )
	, _chargingCurrentLabel( new QLabel( this ) )
	, _bat1tempLabel( new QLabel( this ) )
	, _bat2tempLabel( new QLabel( this ) )
	, _stateLabel( new QLabel( this ) )
	, _versionLabel( new QLabel( this ) )
	, _errorLabel( new QLabel( this ) )
{
	this->setChargerNumber( number );

	QFormLayout* layout = new QFormLayout;
	setLayout( layout );

	layout->addRow( "Time", _timeLabel );
	layout->addRow( "Battery voltage", _batteryVoltageLabel );
	layout->addRow( "Charging current", _chargingCurrentLabel );
	layout->addRow( "Battery 1 Temp", _bat1tempLabel );
	layout->addRow( "Battery 2 Temp", _bat2tempLabel );
	layout->addRow( "State", _stateLabel );
	layout->addRow( "Version", _versionLabel );
	layout->addRow( "Error", _errorLabel );

	QPushButton* clearErrorButton = new QPushButton( "Clear error", this );

	layout->addRow( clearErrorButton );


	bool ok = connect( clearErrorButton, SIGNAL( clicked() ), SLOT( on_clearErrorButton_clicked() ) );
	Q_ASSERT( ok );
}

void ChargerWidget::chargerInfoChanged( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const char* state )
{
	_timeLabel->setText( QString("%1s").arg( time ) );
	_batteryVoltageLabel->setText( QString("%1V").arg( batteryVoltage ) );
	_chargingCurrentLabel->setText( QString("%1A").arg( chargingCurrent ) );
	_bat1tempLabel->setText( QString("%1°C").arg( bat1temp ) );
	_bat2tempLabel->setText( QString("%1°C").arg( bat2temp ) );
	_stateLabel->setText( QString("#%1 %2").arg( state_number ).arg( state ) );
}

void ChargerWidget::chargerErrorChanged( unsigned int time, const char* message )
{
	_timeLabel->setText( QString("%1s").arg( time ) );
	_errorLabel->setText( message );
}

void ChargerWidget::chargerVersionChanged( int major, int minor, int patch )
{
	_versionLabel->setText( QString("%1.%2.%3").arg( major ).arg( minor ).arg( patch ) );
}

void ChargerWidget::on_clearErrorButton_clicked()
{
	this->clearError();
}
