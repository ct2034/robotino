//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _CAMERASETTINGS_H_
#define _CAMERASETTINGS_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/CameraCapabilities.h>

class MyCamera;
class FormatList;

class CameraSettings : public QWidget
{
	Q_OBJECT
public:
	CameraSettings( QWidget* parent, MyCamera* camera );

private Q_SLOTS:
	void on_cameraSelector_activated( int );

	void on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& capablities );

	void on_brightness_valueChanged( int );
	void on_contrast_valueChanged( int );
	void on_saturation_valueChanged( int );
	void on_autoWhiteBalance_valueChanged( int );
	void on_gain_valueChanged( int );
	void on_whiteBalanceTemperature_valueChanged( int );
	void on_backlightCompensation_valueChanged( int );
	void on_autoExposure_valueChanged( int );
	void on_exposure_valueChanged( int );
	void on_autoFocus_valueChanged( int );
	void on_focus_valueChanged( int );
	void on_sharpness_valueChanged( int );

private:
	MyCamera* _camera;

	QComboBox* _cameraSelector;

	FormatList* _formatList;

	QSpinBox* _brightness;
	QSpinBox* _contrast;
	QSpinBox* _saturation;
	QSpinBox* _autoWhiteBalance;
	QSpinBox* _gain;
	QSpinBox* _whiteBalanceTemperature;
	QSpinBox* _backlightCompensation;
	QSpinBox* _autoExposure;
	QSpinBox* _exposure;
	QSpinBox* _autoFocus;
	QSpinBox* _focus;
	QSpinBox* _sharpness;
};

#endif //_CAMERASETTINGS_H_
