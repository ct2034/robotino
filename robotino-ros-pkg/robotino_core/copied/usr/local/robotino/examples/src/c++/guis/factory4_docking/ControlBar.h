//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _ControlBar_H_
#define _ControlBar_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class ControlBar : public QToolBar
{
	Q_OBJECT
public:
	ControlBar( QWidget* parent );

Q_SIGNALS:
	void start( int belt );
	void stop();

private Q_SLOTS:
	void on_startButton_clicked();
	void on_stopButton_clicked();

private:
	QPushButton* _startButton;
	QPushButton* _stopButton;
	QSpinBox* _beltSelector;
};

#endif
