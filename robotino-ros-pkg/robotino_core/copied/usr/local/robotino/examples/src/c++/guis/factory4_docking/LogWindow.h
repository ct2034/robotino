#ifndef _REC_QT_LOG_WINDOW_H_
#define _REC_QT_LOG_WINDOW_H_

#include <QtCore>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif


class LogWindow : public QDockWidget
{
	Q_OBJECT
public:
	LogWindow( QAction* hideShowAction, QWidget* parent );

	void updateAction();

private:
	void showEvent( QShowEvent* e );
	void hideEvent( QHideEvent* e );

	QAction* _hideShowAction;

	private Q_SLOTS:
		void on_logmessage_triggered();
};

#endif //_REC_QT_LOG_WINDOW_H_

