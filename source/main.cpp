#include "main.h"
#include <c4d.h>


Bool PluginStart()
{
	
	if (!RegisterFirstObject()) return false;
	

	return true;
}

void PluginEnd()
{
}

Bool PluginMessage(Int32 id, void* data)
{
	//return true;

	switch (id)
	{
	case C4DPL_INIT_SYS:
		if (!g_resource.Init())
			return false;		// don't start plugin without resource
		return true;
		break;
	}

	return false;
}

