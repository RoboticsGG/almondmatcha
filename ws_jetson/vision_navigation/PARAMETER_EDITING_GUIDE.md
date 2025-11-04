# Parameter Editing Guide: Launch File vs Config.py

## Quick Answer

| Situation | Edit | Reason |
|-----------|------|--------|
| **Quick tuning during rover testing** | Launch file arguments | No rebuild needed, instant changes with `ros2 launch` commands |
| **Permanent default values** | config.py | New developers use sensible defaults, reduce launch command complexity |
| **Development/experimentation** | Both (launch overrides) | config.py + launch args gives maximum flexibility |
| **Production deployment** | config.py | Single source of truth, no launch arguments to remember |

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    YOUR ROVER TUNING SESSION                    │
└─────────────────────────────────────────────────────────────────┘
                                 │
                    ┌────────────┴────────────┐
                    │                         │
            ┌───────▼────────┐      ┌────────▼──────────┐
            │  Launch File   │      │    config.py      │
            │  (Immediate)   │      │  (Requires rebuild)│
            └───────┬────────┘      └────────┬──────────┘
                    │                         │
        ┌───────────┴────────────┬────────────┴──────────────┐
        │                        │                           │
   ✓ Easy to change        ✓ Single source      ✓ Fallback defaults
   ✓ No rebuild            ✓ IDE autocomplete    ✓ Python defaults
   ✗ Session-only          ✗ Requires rebuild   ✗ Requires build
```

---

## Parameter Priority (Data Flow)

```
Launch File Arguments (HIGHEST PRIORITY)
        ↓
        └─→ Overrides node parameter servers
        └─→ Overrides config.py defaults
        └─→ Runtime values in executable
                ↓
          config.py (FALLBACK DEFAULTS)
                ↓
          Hardcoded in Python code (NEVER DO THIS)
```

**Key Principle**: Launch arguments take precedence over config.py. If you pass a value via `ros2 launch`, it overrides config.py completely.

---

## Use Case Scenarios

### Scenario 1: Tuning Rover Steering During Session ⚡

**Goal**: Test different PID gains without rebuild

**Solution**: Use launch file arguments

```bash
# Try PID gains during tuning
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=5.0 k_i:=0.1 k_d:=0.2

# Test different steering sensitivity
ros2 launch vision_navigation vision_navigation.launch.py \
  k_e1:=1.5 k_e2:=0.05

# Try max steering angle
ros2 launch vision_navigation vision_navigation.launch.py \
  steer_max_deg:=45.0
```

**Why this is easy**:
- ✓ No rebuild needed
- ✓ Changes apply immediately when you relaunch
- ✓ Multiple attempts in minutes, not hours
- ✓ Keep session notes for what works

**When to use**: Every tuning session! Rover testing, parameter experimentation, quick validation.

---

### Scenario 2: Finding "Good Defaults" → Permanent Config ✅

**Goal**: After tuning, save best parameters as defaults

**Timeline**:
1. Tuning session: Use launch arguments to test values
2. Found best values: `k_p=4.5`, `k_i=0.1`, `k_d=0.15`
3. Save to config.py for future sessions

**Solution**: Edit config.py and rebuild

```python
# config.py - AFTER tuning session
class ControlConfig:
    K_P = 4.5        # ← Updated from 4.0 (best from testing)
    K_I = 0.1        # ← Updated from 0.0 (stable damping)
    K_D = 0.15       # ← Updated from 0.0 (responsive)
```

```bash
# Rebuild for new defaults
colcon build --packages-select vision_navigation

# Now launch without arguments uses new defaults
ros2 launch vision_navigation vision_navigation.launch.py
```

**Why this flow**:
- ✓ Tuning is fast (launch file)
- ✓ Defaults are saved (config.py)
- ✓ New team members get good starting point
- ✓ Fewer launch arguments to remember

---

### Scenario 3: Production Deployment 🚀

**Goal**: System runs consistently with zero command-line tuning

**Solution**: All values in config.py, no launch arguments needed

```bash
# Simple, clean, no parameters to remember
ros2 launch vision_navigation vision_navigation.launch.py
```

**Result**: System uses all values from config.py as-is.

---

## Practical Tuning Workflow

### Phase 1: Initial Tuning (First Session)

```
1. Read config.py to understand current values
   ↓
2. Create launch command with experimental parameters
   ros2 launch vision_navigation vision_navigation.launch.py k_p:=5.0
   ↓
3. Test on rover, record performance
   ↓
4. Adjust values, relaunch (no rebuild!)
   ros2 launch vision_navigation vision_navigation.launch.py k_p:=5.5
   ↓
5. Test again, iterate until satisfied
```

**Key**: Each iteration takes 10-30 seconds, not 2-5 minutes (rebuild time).

---

### Phase 2: Save Best Values

```
1. Document what worked:
   "Best gains: k_p=4.5, k_i=0.1, k_d=0.15"
   ↓
2. Edit config.py with new values
   ↓
3. Rebuild (one-time cost)
   colcon build --packages-select vision_navigation
   ↓
4. Test with new defaults
   ros2 launch vision_navigation vision_navigation.launch.py
```

**Key**: Rebuild happens once, saves time for all future sessions.

---

### Phase 3: Future Sessions

```
1. Launch uses config.py defaults (already tuned)
   ros2 launch vision_navigation vision_navigation.launch.py
   ↓
2. If minor tweaks needed, use launch arguments
   ros2 launch vision_navigation vision_navigation.launch.py k_p:=4.6
   ↓
3. If major changes needed, update config.py and rebuild
```

---

## Parameter Categories & Where to Edit

### 📸 Camera Parameters

| Parameter | config.py | Launch File | Notes |
|-----------|-----------|-------------|-------|
| WIDTH, HEIGHT | ✓ | `camera_width`, `camera_height` | Set once, rarely changes |
| FPS | ✓ | `camera_fps` | Tuning: use launch arg |
| PREVIEW | ✓ | `camera_preview` | Debugging: use launch arg |
| VIDEO_PATH | ✓ | `video_path` | Testing: use launch arg |

**Recommendation for tuning**: Use launch args for FPS and preview testing.

---

### 🎯 Lane Detection Parameters

| Parameter | config.py | Launch File | Notes |
|-----------|-----------|-------------|-------|
| Color thresholds | ✓ | ✗ | Change in config.py → rebuild |
| Gradient thresholds | ✓ | ✗ | Change in config.py → rebuild |
| Window parameters | ✓ | ✗ | Change in config.py → rebuild |
| Visualization | ✓ | `lane_visualization` | Debugging: use launch arg |

**Recommendation for tuning**: Lane detection tuning requires rebuild (in config.py). Use visualization launch arg for debugging.

---

### 🎮 Control (Steering) Parameters ⭐

| Parameter | config.py | Launch File | Notes |
|-----------|-----------|-------------|-------|
| K_P, K_I, K_D | ✓ | `k_p`, `k_i`, `k_d` | **Always use launch args for tuning!** |
| K_E1, K_E2 | ✓ | `k_e1`, `k_e2` | **Always use launch args for tuning!** |
| EMA_ALPHA | ✓ | `ema_alpha` | **Always use launch args for tuning!** |
| STEER_MAX_DEGREES | ✓ | `steer_max_deg` | **Always use launch args for tuning!** |
| INTEGRAL_LIMIT | ✓ | ✗ | Rarely changed, edit config.py |

**⭐ IMPORTANT**: Control parameters should ALWAYS be tuned via launch file arguments. This is where launch file adds the most value.

---

## Tuning Command Templates

### Test Different PID Gains
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=3.5 k_i:=0.05 k_d:=0.1
```

### Test Different Error Weights
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_e1:=1.2 k_e2:=0.15
```

### Test Smoothing Factor
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  ema_alpha:=0.1
```

### Test Maximum Steering Angle
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  steer_max_deg:=45.0
```

### Test with Video File Input
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  video_path:=/home/yupi/test_video.mp4 \
  camera_preview:=true
```

### Combine Multiple Parameters
```bash
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=4.5 k_i:=0.1 k_d:=0.15 \
  k_e1:=1.3 k_e2:=0.12 \
  ema_alpha:=0.08 \
  steer_max_deg:=55.0
```

---

## When to Rebuild config.py

**Rebuild needed when**:
- ✓ Found optimal parameters after tuning (save as defaults)
- ✓ Adding new configuration parameters
- ✓ Changing hardware setup (camera, rover dimensions, etc.)
- ✓ Deploying to production (finalize all values)

**Don't rebuild when**:
- ✗ Quick testing (use launch args instead)
- ✗ One-time experiments (use launch args instead)
- ✗ Different rover configurations (create separate config variants)

---

## Priority Decision Tree

```
Are you tuning parameters RIGHT NOW?
    ├─ YES → Use launch file arguments
    │         (No rebuild, instant testing)
    │
    └─ NO → Found good values to keep?
              ├─ YES → Update config.py + rebuild
              │         (Save as new defaults)
              │
              └─ NO → Avoid editing
                       (Use defaults or launch args)
```

---

## Summary Table

| Task | How | Time | Effort |
|------|-----|------|--------|
| Quick PID tuning | `ros2 launch ... k_p:=X` | 10s | Easy |
| Try camera settings | `ros2 launch ... camera_fps:=X` | 10s | Easy |
| Debug visualization | `ros2 launch ... lane_visualization:=true` | 10s | Easy |
| Save tuned values | Edit config.py + `colcon build` | 30-60s | Medium |
| Change lane detection algorithm | Edit config.py + `colcon build` | 30-60s | Medium |
| Production deployment | Finalize config.py, no launch args | 0s | Easy |

---

## Pro Tips for Rover Tuning

### 💡 Tip 1: Keep a Tuning Log
```bash
# Document what you tried
k_p=4.0  → Understeers on sharp turns
k_p=5.0  → Good response, slight oscillation
k_p=4.5  → ✓ BEST (balanced)
```

### 💡 Tip 2: Tune One Parameter at a Time
```bash
# Good: Test only k_p changes
ros2 launch vision_navigation vision_navigation.launch.py k_p:=4.5
ros2 launch vision_navigation vision_navigation.launch.py k_p:=5.0

# Bad: Change multiple at once (can't tell what helped)
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=5.0 k_i:=0.1 k_d:=0.15 ema_alpha:=0.1
```

### 💡 Tip 3: Document Command for Next Time
After finding good parameters, save the command:
```bash
# Save to a file or notes
# BEST CONFIG FROM NOV 4 2025:
ros2 launch vision_navigation vision_navigation.launch.py \
  k_p:=4.5 k_i:=0.1 k_d:=0.15 \
  k_e1:=1.3 k_e2:=0.12 ema_alpha:=0.08
```

### 💡 Tip 4: Use Launch Arguments for A/B Testing
```bash
# Terminal 1: Test version A
ros2 launch vision_navigation vision_navigation.launch.py k_p:=4.0

# Terminal 2: Test version B (side-by-side comparison)
ros2 launch vision_navigation vision_navigation.launch.py k_p:=5.0
```

---

## Decision: Which File for Each Parameter?

### Rule of Thumb

**Use config.py for**: Fixed hardware, algorithm constants, system setup
- Camera resolution (rarely changes)
- Lane detection thresholds
- Maximum steering angle (rover-specific)
- Node names and topic names

**Use launch arguments for**: Runtime tuning, experimentation, testing
- PID gains (main tuning parameter)
- Error weights
- Smoothing factors
- Visualization flags
- Input sources (video file vs camera)

---

## Comparison: Before vs After This Clarification

### Before (Confusing)
- Where do I edit parameters? Both places?
- Do I need to rebuild every time?
- What takes priority?
- How long is each tuning iteration?

### After (Clear)
- **Quick tuning** (10s): `ros2 launch ... parameter:=value`
- **Save tuned values** (1m): Edit config.py + rebuild
- **Production** (0s): Just launch, no arguments
- **Priority**: Launch args > config.py > hardcoded

---

## Next Steps

1. **For current tuning session**: Use launch file arguments for PID gain tuning
   ```bash
   ros2 launch vision_navigation vision_navigation.launch.py \
     k_p:=4.5 k_i:=0.1 k_d:=0.15
   ```

2. **After finding good values**: Update config.py and rebuild
   ```bash
   # Edit ControlConfig in config.py
   colcon build --packages-select vision_navigation
   ```

3. **For next session**: Use new defaults
   ```bash
   ros2 launch vision_navigation vision_navigation.launch.py
   # (already has your tuned values)
   ```

---

**Created**: November 4, 2025  
**For**: Rover tuning and parameter management  
**Applies to**: ws_jetson vision_navigation system
