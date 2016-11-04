/******************************************************************************/
/***   Includes                                                             ***/
/******************************************************************************/
#include "plugin/simulation/Interface.h"

#include "Tutorial1.h"

BEGIN_SIMULATION_INTERFACE( "REC GmbH", "Tutorial 1" )
	BEGIN_UNITS
		ADD_UNIT( "REC GmbH Tutorial 1", Tutorial1 )
	END_UNITS
END_SIMULATION_INTERFACE
