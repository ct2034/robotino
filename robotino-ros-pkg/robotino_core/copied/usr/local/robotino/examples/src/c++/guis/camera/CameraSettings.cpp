//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#include "CameraSettings.h"
#include "MyCamera.h"
#include "FormatList.h"

CameraSettings::CameraSettings( QWidget* parent, MyCamera* camera )
	: QWidget( parent )
	, _camera( camera )
	, _cameraSelector( new QComboBox( this ) )
	, _formatList( new FormatList( this, camera ) )
	, _brightness( new QSpinBox( this ) )
	, _contrast( new QSpinBox( this ) )
	, _saturation( new QSpinBox( this ) )
	, _autoWhiteBalance( new QSpinBox( this ) )
	, _gain( new QSpinBox( this ) )
	, _whiteBalanceTemperature( new QSpinBox( this ) )
	, _backlightCompensation( new QSpinBox( this ) )
	, _autoExposure( new QSpinBox( this ) )
	, _exposure( new QSpinBox( this ) )
	, _autoFocus( new QSpinBox( this ) )
	, _focus( new QSpinBox( this ) )
	, _sharpness( new QSpinBox( this ) )
{
	_cameraSelector->addItems( QStringList() << "none" << "Camera 0" << "Camera 1" << "Camera 2" << "Camera 3" );

	QVBoxLayout* layout = new QVBoxLayout;
	setLayout( layout );

	layout->addWidget( _formatList );

	QFormLayout* flayout = new QFormLayout;
	layout->addLayout( flayout );
	flayout->addRow( "camera", _cameraSelector );
	flayout->addRow( "brightness", _brightness );
	flayout->addRow( "contrast", _contrast );
	flayout->addRow( "saturation", _saturation );
	flayout->addRow( "autoWhiteBalance", _autoWhiteBalance );
	flayout->addRow( "gain", _gain );
	flayout->addRow( "whiteBalanceTemperature", _whiteBalanceTemperature );
	flayout->addRow( "backlightCompensation", _backlightCompensation );
	flayout->addRow( "autoExposure", _autoExposure );
	flayout->addRow( "exposure", _exposure );
	flayout->addRow( "autoFocus", _autoFocus );
	flayout->addRow( "focus", _focus );
	flayout->addRow( "sharpness", _sharpness );

	_brightness->setRange( 0, 0xFFFF );
	_contrast->setRange( 0, 0xFFFF );
	_saturation->setRange( 0, 0xFFFF );
	_autoWhiteBalance->setRange( 0, 1 );
	_gain->setRange( 0, 0xFFFF );
	_whiteBalanceTemperature->setRange( 0, 0xFFFF );
	_backlightCompensation->setRange( 0, 0xFFFF );
	_autoExposure->setRange( 0, 1 );
	_exposure->setRange( 0, 0xFFFF );
	_autoFocus->setRange( 0, 1 );
	_focus->setRange( 0, 0xFFFF );
	_sharpness->setRange( 0, 0xFFFF );

	_brightness->setEnabled( false );
	_contrast->setEnabled( false );
	_saturation->setEnabled( false );
	_autoWhiteBalance->setEnabled( false );
	_gain->setEnabled( false );
	_whiteBalanceTemperature->setEnabled( false );
	_backlightCompensation->setEnabled( false );
	_autoExposure->setEnabled( false );
	_exposure->setEnabled( false );
	_autoFocus->setEnabled( false );
	_focus->setEnabled( false );
	_sharpness->setEnabled( false );

	bool ok = connect( _cameraSelector, SIGNAL( activated( int ) ), SLOT( on_cameraSelector_activated( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _camera, SIGNAL( capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& ) ), SLOT( on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& ) ) );
	Q_ASSERT( ok );

	ok = connect( _brightness, SIGNAL( valueChanged( int ) ), SLOT( on_brightness_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _contrast, SIGNAL( valueChanged( int ) ), SLOT( on_contrast_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _saturation, SIGNAL( valueChanged( int ) ), SLOT( on_saturation_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _autoWhiteBalance, SIGNAL( valueChanged( int ) ), SLOT( on_autoWhiteBalance_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _gain, SIGNAL( valueChanged( int ) ), SLOT( on_gain_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _whiteBalanceTemperature, SIGNAL( valueChanged( int ) ), SLOT( on_whiteBalanceTemperature_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _backlightCompensation, SIGNAL( valueChanged( int ) ), SLOT( on_backlightCompensation_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _autoExposure, SIGNAL( valueChanged( int ) ), SLOT( on_autoExposure_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _exposure, SIGNAL( valueChanged( int ) ), SLOT( on_exposure_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _autoFocus, SIGNAL( valueChanged( int ) ), SLOT( on_autoFocus_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _focus, SIGNAL( valueChanged( int ) ), SLOT( on_focus_valueChanged( int ) ) );
	Q_ASSERT( ok );

	ok = connect( _sharpness, SIGNAL( valueChanged( int ) ), SLOT( on_sharpness_valueChanged( int ) ) );
	Q_ASSERT( ok );
}

void CameraSettings::on_cameraSelector_activated( int index )
{
	_camera->setCameraNumber( index - 1 );
}

void CameraSettings::on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& capablities )
{
	_brightness->setEnabled( capablities.brightness );
	_contrast->setEnabled( capablities.contrast );
	_saturation->setEnabled( capablities.saturation );
	_autoWhiteBalance->setEnabled( capablities.autoWhiteBalance );
	_gain->setEnabled( capablities.gain );
	_whiteBalanceTemperature->setEnabled( capablities.whiteBalanceTemperature );
	_backlightCompensation->setEnabled( capablities.backlightCompensation );
	_autoExposure->setEnabled( capablities.autoExposure );
	_exposure->setEnabled( capablities.exposure );
	_autoFocus->setEnabled( capablities.autoFocus );
	_focus->setEnabled( capablities.focus );
	_sharpness->setEnabled( capablities.sharpness );
}

void CameraSettings::on_brightness_valueChanged( int value )
{
	_camera->setBrightness( value );
}

void CameraSettings::on_contrast_valueChanged( int value )
{
	_camera->setContrast( value );
}

void CameraSettings::on_saturation_valueChanged( int value )
{
	_camera->setSaturation( value );
}

void CameraSettings::on_autoWhiteBalance_valueChanged( int value )
{
	_camera->setAutoWhiteBalanceEnabled( value > 0 );
}

void CameraSettings::on_gain_valueChanged( int value )
{
	_camera->setGain( value );
}

void CameraSettings::on_whiteBalanceTemperature_valueChanged( int value )
{
	_camera->setWhiteBalanceTemperature( value );
}

void CameraSettings::on_backlightCompensation_valueChanged( int value )
{
	_camera->setBacklightCompensation( value );
}

void CameraSettings::on_autoExposure_valueChanged( int value )
{
	_camera->setAutoExposureEnabled( value > 0 );
}

void CameraSettings::on_exposure_valueChanged( int value )
{
	_camera->setExposure( value );
}

void CameraSettings::on_autoFocus_valueChanged( int value )
{
	_camera->setAutoFocusEnabled( value > 0 );
}

void CameraSettings::on_focus_valueChanged( int value )
{
	_camera->setFocus( value );
}

void CameraSettings::on_sharpness_valueChanged( int value )
{
	_camera->setSharpness( value );
}
