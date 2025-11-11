import hou
import piston

def pistonClearProfiler(event_type, **kwargs):
	if event_type == hou.hipFileEventType.AfterClear:
		piston.clearProfiler()

hou.hipFile.addEventCallback(pistonClearProfiler)