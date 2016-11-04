//  Copyright (C) 2004-2009, Robotics Equipment Corporation GmbH

#ifndef _ADDRESSINPUT_H_
#define _ADDRESSINPUT_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class AddressInput : public QToolBar
{
	Q_OBJECT
public:
	AddressInput( QWidget* parent );

private Q_SLOTS:
	void on_connectButton_clicked();
	void on_connectTimer_timeout();

	void on_com_connected();
	void on_com_disconnected();

private:
	QLineEdit* _lineEdit;
	QPushButton* _connectButton;
	QTimer* _connectTimer;

	int _connectCounter;
};

#endif
