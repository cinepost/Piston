import hou
import _piston

def pistonClearProfiler(event_type, **kwargs):
	if event_type == hou.hipFileEventType.AfterClear:
		_piston.DeformerFactory.clear()
		_piston.clearProfiler()

hou.hipFile.addEventCallback(pistonClearProfiler)