import hou
import piston

try:
	from PySide6 import QtCore
except:
	QtCore = None

from piston_houdini.shelf_tools import pistonUpdateCacheShelfButton

def pistonCallbacks(event_type, **kwargs):
	if event_type == hou.hipFileEventType.AfterClear:
		# triggered immediately after the current .hip file is cleared
		piston.DeformerFactory.clear()
		piston.clearProfiler()

	if event_type == hou.hipFileEventType.BeforeLoad:
		# when new .hip file is about to be loaded we should delete all existing deformers from factory
		piston.DeformerFactory.clear()

hou.hipFile.addEventCallback(pistonCallbacks)

pistonUpdateCacheShelfButton()

if not hasattr(hou.session, 'piston_shelf_global_cache_icon_timer'):
	if QtCore:
		hou.session.icon_timer = QtCore.QTimer()
		hou.session.icon_timer.timeout.connect(pistonUpdateCacheShelfButton)
		hou.session.icon_timer.start(200) # Interval in milliseconds
	else:
		hou.ui.addEventLoopCallback(pistonUpdateCacheShelfButton)