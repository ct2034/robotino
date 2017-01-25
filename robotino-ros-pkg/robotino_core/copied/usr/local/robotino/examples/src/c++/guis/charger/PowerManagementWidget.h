#ifndef _PowerManagementWidget_H_
#define _PowerManagementWidget_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/all.h>

class PowerManagementWidget : public QWidget, rec::robotino::api2::PowerManagement
{
	Q_OBJECT
public:
	PowerManagementWidget();

private:
	void readingsEvent( float battery_voltage, float system_current, bool ext_power, int num_chargers, const char* batteryType, bool batteryLow, int batteryLowShutdownCounter );

	QLabel* _battery_voltageLabel;
	QLabel* _system_currentLabel;
	QLabel* _ext_powerLabel;
	QLabel* _num_chargersLabel;
	QLabel* _batteryTypeLabel;
	QLabel* _batteryLowLabel;
	QLabel* _batteryLowShutdownCounterLabel;
};

#endif
