# Piston Library

A C++/Python library for high-performance geometry deformation and processing.

## Installation

### Houdini Integration

To enable **Piston** in Houdini, you must update your `Houdini.env` file (usually located in `~/houdiniX.X/houdini.env`).

Add the following line to ensure Houdini can locate the Piston plugins, OTLs, and shelf files:

#### Add Piston to the Houdini Path

**Linux:**
```bash
HOUDINI_PATH = "/opt/piston/houdini_${HOUDINI_VERSION};&"
```
**Windows:**
```bash
HOUDINI_PATH = "D:\work\piston\houdini_${HOUDINI_VERSION};&"
```

---

## Configuration

### Environment Variables

#### `PISTON_PTCACHE`
Initializes the default state of the interactive points cache when the `piston` module is first loaded.

*   **Supported Values:** `ON`/`OFF`, `TRUE`/`FALSE`, or `1`/`0`.
*   **Behavior:** This variable is **case-insensitive** (e.g., `off`, `Off`, and `OFF` are all valid).
*   **Usage:** Setting this to `OFF` or `0` will disable global caching by default, which can be useful for automated testing or environments with limited memory.

#### `PISTON_DEFAULT_TPOSE_FRAME`
Specifies the exact timeline frame where T-pose geometry and rest-state attributes are captured. 

*   **Format:** Accepts `int`, `float`, or `double` values to support precise sub-frame sampling.
*   **Usage:** This variable is utilized if no specific "rest" attribute name is provided.
*   **Default:** If unset, the library defaults to **frame 0.0** or the first available frame in the sequence.

#### `PISTON_DEFAULT_DATA_PRIM_PATH`
Sets the default USD primitive path used by the `piston` module for deformer data storage.

*   **Default Value:** `/__piston_data__`
*   **Behavior:** Defines the fallback location where Piston will look for or create data storage primitives. 
*   **Usage:** This path is used automatically unless a different path is explicitly defined via the `setDataPrimPath()` method on a specific deformer instance. Use this variable to redirect data storage to a specific namespace or hidden hierarchy within your USD stage.

#### `PISTON_DATA_TO_PRIM_STORAGE`
This environment variable determines how Piston stores data on a Pixar USD primitive.

*   **Supported Values:** `attribute` or `metadata`.
*   **Default Value:** `attribute`.
*   **Behavior:** This variable is **case-insensitive**. 
    *   Setting this to `attribute` writes data as a standard USD attribute. 
    *   Setting this to `metadata` writes the data as custom metadata on the primitive.

#### `PISTON_DATA_INSTANCING`
Initializes the default state of the interactive points cache when the `piston` module is first loaded.

*   **Supported Values:** `ON`/`OFF`, `TRUE`/`FALSE`, or `1`/`0`.
*   **Behavior:** This variable is **case-insensitive** (e.g., `off`, `Off`, and `OFF` are all valid).
*   **Usage:** Setting this to `OFF` or `0` will disable defromers instancing globally.
    
---

## Logging

The `piston` module includes a built-in logging system to control console output and diagnostic verbosity. 

### Setting the Log Level
Use the `piston.setLogLevel()` function to adjust how much information is displayed.

```python
import piston

# Example: Set the logging level to DEBUG for troubleshooting
piston.setLogLevel(piston.LogLevel.DEBUG)
```

### Available Logging Levels

Use these constants with `piston.setLogLevel()` to control output verbosity:

| Level | Description |
| :--- | :--- |
| `piston.LogLevel.TRACE` | Exhaustive internal details for deep troubleshooting. |
| `piston.LogLevel.DEBUG` | Detailed information useful for diagnosing issues. |
| `piston.LogLevel.INFO` | General confirmation of normal application behavior. |
| `piston.LogLevel.WARNING` | Indications of unexpected events or potential future problems. |
| `piston.LogLevel.ERROR` | Serious issues that prevented a specific function from succeeding. |
| `piston.LogLevel.FATAL` | Critical failures that lead to immediate termination. |

---

## Interactive Points Cache

The **piston** module includes an internal caching system for deformed geometry points. Its primary purpose is to store computed positions so users can scrub the timeline with higher **FPS** and better responsiveness, especially when working with large or heavy geometries.

### Global Cache Control
You can toggle the interactive cache for the entire **piston** module at any time. This is useful for global performance tuning or debugging.

*   **Houdini Integration:** A **tool button** is available in the **Piston shelf** within Houdini to quickly toggle the global cache state on or off.

```python
import piston

# Toggle the cache globally (True = On, False = Off)
piston.DeformerFactory.setPointsCacheUsageState(True)
```

## Per-Deformer Cache Control
For more granular control, the cache can be switched off or on for individual deformers. 
This allows you to keep caching active for the majority of your scene while disabling it for specific geometries.

```python
# Toggle the cache for a specific deformer instance
deformer.setPointsCacheUsageState(False)
```

Note: While the cache significantly improves playback speed, disabling it ensures that geometry is recomputed every frame, which can be helpful for verifying live deformation logic.

---

## Utility Methods

The **piston** module provides several utility functions for managing the global state and cleaning up resources.

### `piston.clearAll()`
This method performs a complete reset of the **piston** environment. It is particularly useful when switching scenes or starting a new session within the same process.

*   **Action:** Removes all active deformers, clears the associated geometry data, and flushes any stored interactive points cache.
*   **Houdini Integration:** For convenience, there is a **tool button** located in the **Piston shelf** within Houdini that calls this method directly.
*   **Usage:**
    ```python
    import piston

    # Reset the module to its initial state
    piston.clearAll()
    ```
---
