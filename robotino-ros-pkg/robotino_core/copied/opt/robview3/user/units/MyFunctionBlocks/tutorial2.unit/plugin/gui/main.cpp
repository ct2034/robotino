/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "plugin/gui/Interface.h"

#include "Tutorial2.h"

BEGIN_GUI_INTERFACE( "REC GmbH", "Tutorial 2" )
	BEGIN_UNITWIDGETS
		ADD_UNITWIDGET("REC GmbH Tutorial 2", "dialog", Tutorial2 )
	END_UNITWIDGETS
END_GUI_INTERFACE
