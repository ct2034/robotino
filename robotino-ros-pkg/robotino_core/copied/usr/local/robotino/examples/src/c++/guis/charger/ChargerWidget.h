#ifndef _ChargerWidget_H_
#define _ChargerWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class ChargerWidget : public QWidget, rec::robotino::api2::Charger
{
	Q_OBJECT
public:
	ChargerWidget( unsigned int number );

private Q_SLOTS:
	void on_clearErrorButton_clicked();

private:
	void chargerInfoChanged( unsigned int time, float batteryVoltage, float chargingCurrent, float bat1temp, float bat2temp, int state_number, const char* state );
	void chargerErrorChanged( unsigned int time, const char* message );
	void chargerVersionChanged( int major, int minor, int patch );

	QLabel* _timeLabel;
	QLabel* _batteryVoltageLabel;
	QLabel* _chargingCurrentLabel;
	QLabel* _bat1tempLabel;
	QLabel* _bat2tempLabel;
	QLabel* _stateLabel;
	QLabel* _versionLabel;
	QLabel* _errorLabel;
};

#endif
