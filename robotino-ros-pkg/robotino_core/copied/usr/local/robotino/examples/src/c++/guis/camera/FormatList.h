//  Copyright (C) 2004-2011, Robotics Equipment Corporation GmbH

#ifndef _FORMATLIST_H_
#define _FORMATLIST_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif

#include <rec/robotino/api2/CameraCapabilities.h>

class MyCamera;

class FormatList : public QListWidget
{
	Q_OBJECT
public:
	FormatList( QWidget* parent, MyCamera* camera );

private Q_SLOTS:
	void on_itemClicked ( QListWidgetItem* item );

	void on_camera_capabilitiesChanged( const rec::robotino::api2::CameraCapabilities& capablities );
	void on_camera_settingsChanged( unsigned int width, unsigned int height, const QString& format );

private:
	MyCamera* _camera;
};

#endif //_FORMATLIST_H_
