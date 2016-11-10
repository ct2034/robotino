/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "plugin/simulation/Interface.h"

#include "Tutorial2.h"

BEGIN_SIMULATION_INTERFACE( "REC GmbH", "Tutorial 2" )
	BEGIN_UNITS
		ADD_UNIT( "REC GmbH Tutorial 2", Tutorial2 )
	END_UNITS
END_SIMULATION_INTERFACE
