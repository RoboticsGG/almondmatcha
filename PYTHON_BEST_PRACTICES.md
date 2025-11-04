# Python Project Organization - Best Practices Guide

**ws_jetson Vision Navigation System**  
**November 4, 2025**

---

## Table of Contents
1. [Configuration Management](#configuration-management)
2. [Helper Functions Organization](#helper-functions-organization)
3. [Comparison with Other Languages](#comparison-with-other-languages)
4. [Python-Specific Patterns](#python-specific-patterns)
5. [Usage Examples](#usage-examples)
6. [Best Practices Summary](#best-practices-summary)

---

## Configuration Management

### The Problem: Hardcoded Values

**BAD - Hardcoded throughout code:**
```python
def camera_node():
    WIDTH = 1280          # Magic number!
    HEIGHT = 720          # Magic number!
    FPS = 30              # Magic number!
    
def control_node():
    K_P = 4.0             # Magic number!
    K_I = 0.0
    K_D = 0.0
```

**Disadvantages:**
- Parameters scattered across multiple files
- Hard to modify (must find and edit each occurrence)
- Different values in different places (inconsistency)
- No clear documentation of what each parameter means

### The Solution: Centralized Configuration (config.py)

```python
# config.py - Single source of truth
class CameraConfig:
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30

class ControlConfig:
    K_P = 4.0
    K_I = 0.0
    K_D = 0.0

# Usage in any node:
from vision_navigation_pkg.config import CameraConfig, ControlConfig

def camera_node():
    width = CameraConfig.WIDTH
    height = CameraConfig.HEIGHT
    fps = CameraConfig.FPS
```

**Advantages:**
- ✓ Single source of truth
- ✓ Easy to modify (change once)
- ✓ Well-documented
- ✓ Consistent across all nodes
- ✓ Version control friendly (track changes in config.py)

---

## Comparison with Other Languages

### File-Based Configuration Patterns

| Language | Pattern | File Name | How Stored |
|----------|---------|-----------|-----------|
| **C/C++** | Header file | `config.h` | `#define` macros |
| **Java** | Properties class | `Config.java` | `public static final` |
| **C#/.NET** | Config class | `Config.cs` | `const` or `ConfigurationManager` |
| **Python** | Config module | `config.py` | Class attributes or dict |
| **JavaScript** | Config object | `config.js` | `const` or `.env` file |
| **Go** | Struct | `config.go` | Struct fields |

### Python Implementation Options

#### Option 1: Module-level constants (SIMPLEST)
```python
# config.py
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 720
CONTROL_K_P = 4.0
```
✓ Simple, flat structure  
✗ All names at same level (naming conflicts)

#### Option 2: Classes with attributes (ORGANIZED - **RECOMMENDED**)
```python
# config.py - This is what we use!
class CameraConfig:
    WIDTH = 1280
    HEIGHT = 720

class ControlConfig:
    K_P = 4.0
```
✓ Well-organized  
✓ IDE autocomplete works great  
✓ Clear namespace (CameraConfig.WIDTH)

#### Option 3: Dataclasses (TYPED - Python 3.7+)
```python
from dataclasses import dataclass

@dataclass
class CameraConfig:
    width: int = 1280
    height: int = 720
```
✓ Type hints built-in  
✗ Slightly more overhead

#### Option 4: Config file (FLEXIBLE)
```yaml
# config.yaml
camera:
  width: 1280
  height: 720
control:
  kp: 4.0
```
```python
# config.py
import yaml
with open('config.yaml') as f:
    cfg = yaml.safe_load(f)
CAMERA_WIDTH = cfg['camera']['width']
```
✓ Easy to modify without code  
✗ Requires file management

#### Option 5: Environment variables (DEPLOYMENT)
```python
import os
CAMERA_WIDTH = int(os.getenv('CAMERA_WIDTH', '1280'))
```
✓ Good for Docker/containers  
✗ Runtime only, harder to track

### Our Choice: Combination Approach

```python
# Start with defaults from config.py (Option 2)
from vision_navigation_pkg.config import CameraConfig

# Can override with environment variables
import os
if 'CAMERA_WIDTH' in os.environ:
    CameraConfig.WIDTH = int(os.environ['CAMERA_WIDTH'])

# Node can further override with ROS parameters
width = self.get_parameter('width').value or CameraConfig.WIDTH
```

**Why?** Best of all worlds:
- Defaults in code (easy to read)
- Environment overrides (for deployment)
- ROS parameters (runtime tuning)

---

## Helper Functions Organization

### The Problem: Scattered Utilities

**BAD - Utilities scattered in multiple files:**
```
camera_stream_node.py: contains helper_clamp(), helper_resize()
lane_detection_node.py: contains resize_image(), clamp_value()
steering_control_node.py: contains clamp(), limit_value()

# Each module has slightly different implementations!
# Functions are duplicated!
```

### The Solution: Centralized helpers.py

```python
# helpers.py - DRY (Don't Repeat Yourself)

def clamp(value, min_val, max_val):
    """Constrain value to range"""
    return max(min_val, min(value, max_val))

def resize_image(img, width, height):
    """Resize image with aspect ratio"""
    # Implementation here

# Usage in any node:
from vision_navigation_pkg.helpers import clamp, resize_image

steering_angle = clamp(value, -60, 60)
small_img = resize_image(frame, 640, 480)
```

---

## Python-Specific Patterns

### Pattern 1: Module vs Class Organization

**Python allows both patterns:**

```python
# PATTERN A: Flat module with functions
# helpers.py
def clamp(value, min_val, max_val):
    ...

def timer_start():
    ...

# Usage
from helpers import clamp
result = clamp(150, -100, 100)
```

```python
# PATTERN B: Classes for organization
# helpers.py
class Math:
    @staticmethod
    def clamp(value, min_val, max_val):
        ...

class Timing:
    @staticmethod
    def timer_start():
        ...

# Usage
from helpers import Math, Timing
result = Math.clamp(150, -100, 100)
```

```python
# PATTERN C: Classes with state (our Timer class)
# helpers.py
class Timer:
    def __init__(self):
        self.start_time = None
    
    def start(self):
        self.start_time = time.time()
    
    def stop(self):
        return time.time() - self.start_time

# Usage
from helpers import Timer
timer = Timer()
timer.start()
# ... do work ...
elapsed = timer.stop()
```

**Recommendation:** 
- Use **Pattern A** (functions) for stateless utilities (clamp, resize, etc.)
- Use **Pattern C** (classes) for stateful utilities (Timer, Logger, etc.)
- Avoid **Pattern B** (static methods in classes) - Python doesn't need this

### Pattern 2: The `__main__` Block

```python
# config.py or helpers.py
if __name__ == "__main__":
    # Only runs when file executed directly
    # NOT when imported
    
    # Perfect for:
    print("Configuration values:")
    print(f"  Camera width: {CameraConfig.WIDTH}")
    print(f"  Control K_P: {ControlConfig.K_P}")
```

**Why?**
- Self-documenting
- Can test module independently
- Shows usage examples
- Never interferes with imports

### Pattern 3: Class Methods vs Static Methods

```python
class CameraConfig:
    WIDTH = 1280
    HEIGHT = 720
    
    # Class method: can modify class state
    @classmethod
    def get_resolution(cls):
        return (cls.WIDTH, cls.HEIGHT)  # Can access cls
    
    # Static method: pure utility, no class access
    @staticmethod
    def is_valid_resolution(width, height):
        return width > 0 and height > 0

# Usage - both work the same
res = CameraConfig.get_resolution()      # Returns (1280, 720)
valid = CameraConfig.is_valid_resolution(640, 480)  # Returns True
```

---

## Usage Examples

### Example 1: Using Config in a Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vision_navigation_pkg.config import CameraConfig, ControlConfig
from vision_navigation_pkg.helpers import clamp, Timer

class CameraStreamNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # Get configuration
        self.width = CameraConfig.WIDTH
        self.height = CameraConfig.HEIGHT
        self.fps = CameraConfig.FPS
        
        # Get control parameters
        self.steer_max = ControlConfig.STEER_MAX_DEGREES
        
        # Use helpers
        self.timer = Timer("frame_processing")
    
    def process_frame(self):
        self.timer.start()
        
        # ... process frame ...
        
        # Clamp steering angle using helper
        steering = clamp(computed_angle, -self.steer_max, self.steer_max)
        
        elapsed = self.timer.stop()
        self.get_logger().info(f"Frame processed in {elapsed*1000:.1f}ms")
```

### Example 2: Modifying Configuration at Runtime

```python
# Option 1: Direct modification (for testing)
from vision_navigation_pkg.config import ControlConfig
ControlConfig.K_P = 3.5  # Tune during testing

# Option 2: Environment override (for deployment)
# In shell:
export CONTROL_K_P=3.5
# In code:
from vision_navigation_pkg.config import override_from_environment
override_from_environment()
# Now ControlConfig.K_P will be 3.5

# Option 3: ROS parameter override (best practice)
# In launch file:
node = ComposableNode(
    package='vision_navigation',
    plugin='CameraNode',
    parameters=[
        {'kp': 3.5},
        {'ki': 0.1},
    ]
)
# In node:
self.kp = self.get_parameter('kp').value or ControlConfig.K_P
```

### Example 3: Adding New Configurations

```python
# In config.py
class NewSystemConfig:
    """New system configuration"""
    PARAMETER_1 = 10
    PARAMETER_2 = 20.5
    
    @classmethod
    def get_parameters(cls):
        return {
            'param1': cls.PARAMETER_1,
            'param2': cls.PARAMETER_2,
        }

# In your node
from vision_navigation_pkg.config import NewSystemConfig
params = NewSystemConfig.get_parameters()
```

---

## Best Practices Summary

### DO ✓

| Practice | Why | Example |
|----------|-----|---------|
| **Centralize configuration** | Single source of truth | Use `config.py` |
| **Use descriptive names** | Self-documenting | `STEER_MAX_DEGREES` not `MAX_STEER` |
| **Add type hints** | IDE support, fewer bugs | `width: int = 1280` |
| **Document parameters** | Understand purposes | Comments explaining each param |
| **Group related configs** | Organization | Separate `CameraConfig`, `ControlConfig` |
| **Use helper functions** | Reusability, DRY | Extract common logic to `helpers.py` |
| **Test edge cases** | Robust code | `clamp()` handles all ranges |
| **Version track changes** | Project history | Git commits for param changes |

### DON'T ✗

| Anti-Pattern | Why Avoid | Better Way |
|--------------|-----------|-----------|
| **Hardcoded magic numbers** | Hard to modify | Use named constants in config.py |
| **Duplicated functions** | Maintenance nightmare | Extract to helpers.py |
| **Mixed file placement** | Confusing organization | One config.py, one helpers.py |
| **Incomplete documentation** | Hard to understand | Docstrings and comments |
| **Type-less code** | Harder to debug | Always add type hints |
| **Inconsistent naming** | Confusing conventions | Use UPPERCASE for constants |

---

## File Structure Reference

```
vision_navigation_pkg/
├── __init__.py
│
├── config.py                    # ← ALL system parameters here
│   ├── CameraConfig
│   ├── LaneDetectionConfig
│   ├── ControlConfig
│   └── SystemConfig
│
├── helpers.py                   # ← ALL helper functions here
│   ├── Conversion helpers (degrees_to_radians)
│   ├── Validation helpers (is_valid_number)
│   ├── Math helpers (clamp, lerp)
│   ├── Image helpers (resize, crop, draw)
│   ├── Logging helpers (setup_csv, log_row)
│   └── Timing helpers (Timer class)
│
├── camera_stream_node.py        # Uses: CameraConfig, helpers
├── lane_detection_node.py       # Uses: LaneDetectionConfig, helpers
├── steering_control_node.py     # Uses: ControlConfig, helpers
│
├── lane_detector.py             # Pure algorithm, no config
├── control_filters.py           # Pure algorithm, no config
│
└── README.md
```

**Import Pattern:**
```python
# In node files
from vision_navigation_pkg.config import CameraConfig, ControlConfig
from vision_navigation_pkg.helpers import clamp, Timer, resize_image
```

---

## Conclusion

**Python Configuration Best Practices:**

1. **Use `config.py`** for all system parameters
   - Organized in classes by subsystem
   - Type hints included
   - Well documented

2. **Use `helpers.py`** for reusable functions
   - Stateless utilities as functions
   - Stateful utilities as classes
   - Comprehensive docstrings

3. **Pattern:** Config → Node → Algorithm
   - Nodes read config
   - Nodes call helpers
   - Algorithm modules stay pure

4. **Development Workflow:**
   - Modify parameters in `config.py`
   - Test with command-line overrides
   - Deploy with environment variables or ROS parameters

This approach provides:
- ✓ Centralized management
- ✓ Easy maintenance
- ✓ Team collaboration
- ✓ Professional code organization
- ✓ Python best practices compliance
