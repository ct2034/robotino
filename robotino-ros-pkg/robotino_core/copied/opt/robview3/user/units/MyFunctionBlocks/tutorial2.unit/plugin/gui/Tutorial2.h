#ifndef _Tutorial2_H_
#define _Tutorial2_H_

/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "plugin/gui/UnitDialog.h"

/******************************************************************************/
/***   Declaration of class CounterDialog                                   ***/
/******************************************************************************/
class Tutorial2 : public plugin::gui::UnitDialog
{
	Q_OBJECT
public:
	Tutorial2 (plugin::gui::UnitDelegate& del );

	void update (const plugin::gui::UnitHistoryBundle& buf);
	void translate (void);

private Q_SLOTS:
	void on_n_valueChanged( int );

private:
	QSpinBox* _n;
};

#endif //_Tutorial2_H_

