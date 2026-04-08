import hou
import piston

def pistonUpdateCacheShelfButton():
	tool = hou.shelves.tool("ptCacheToggle")
	global_cache_state = piston.DeformerFactory.getPointsCacheUsageState()

	if global_cache_state:
		tool.setIcon("Piston_PtCache_On")
	else:
		tool.setIcon("Piston_PtCache_Off")

def clearAll():
	piston.clearAll()

def toggleGlobalPointsCache():
	global_cache_state = piston.DeformerFactory.getPointsCacheUsageState()
	piston.DeformerFactory.setPointsCacheUsageState(not global_cache_state)
	pistonUpdateCacheShelfButton()
