import hou
import piston

def pistonCallbacks(event_type, **kwargs):
	if event_type == hou.hipFileEventType.AfterClear:
		# triggered immediately after the current .hip file is cleared
		piston.DeformerFactory.clear()
		piston.clearProfiler()

	if event_type == hou.hipFileEventType.BeforeLoad:
		# when new .hip file is about to be loaded we should delete all existing deformers from factory
		piston.DeformerFactory.clear()

hou.hipFile.addEventCallback(pistonCallbacks)