
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_ARDUPILOT
#define __UVX_ARDUPILOT

#include "uvx_universal_types.h"

typedef struct UVX_ARDUPILOT
{
	// Ardupilot status flags
	U8 				Initialised;
	U8 				Armed;
	U8 				Land_Complete;
	U8 				Flying;
	U8				Flag5;
	U8				Flag6;
	U8				Flag7;
	U8				PSYS_Armed;
	
	// Notify LED RGB values vector
	VECTOR_3D_U8 	RGB_LED_Color;

} UVX_ARDUPILOT;

#endif
