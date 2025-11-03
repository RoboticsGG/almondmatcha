# Tomorrow's Action Items - Quick Start

## Build & Test (15–20 min)

### Step 1: Build
```bash
cd /home/yupi/almondmatcha/mros2-mbed-chassis-dynamics
./build.bash all
# Check for errors; if docker issue, try native or troubleshoot daemon
```

### Step 2: If Build Succeeds
- Find the `.bin` file in the build output
- Flash to STM32 Nucleo via serial/SWD debugger
- Open serial console (115200 baud) to monitor logs

### Step 3: Hardware Verification
- [ ] LED1 OFF during startup (~1 second)
- [ ] LED1 ON solid after "All initialization complete"
- [ ] Serial console shows: "Status LED turned on solid (ready to operate)"
- [ ] Motor responds to ROS2 commands (test via `ros2 topic pub`)
- [ ] IMU data published to `tp_imu_data_d5` @ 10 Hz

---

## Code Review Checklist

- [ ] Review `REFACTORING_SUMMARY.md` in workspace/chassis_controller/
- [ ] Verify each module's responsibility is clear
- [ ] Check that extern/global definitions match headers
- [ ] Confirm no duplicate symbols across files

---

## If Everything Works
1. Commit modular refactoring:
   ```bash
   git add -A
   git commit -m "refactor: Modularize chassis controller (motor_control + led_status)"
   git push origin main
   ```

2. Update main README with new file structure

3. Document in WORK_SESSION_2025-11-04.md results

---

## If Build Fails

### Docker Issue
- Check: `docker ps` (does docker daemon run?)
- Try: `sudo dockerd` or docker restart
- Alternative: Use native build (cross-check with team)

### Compilation Error
- Check error message in terminal output
- Most likely: Missing include, typo in extern, or Mbed OS version mismatch
- Solution: Review the specific file and line number, compare with original

### Linker Error (symbol already defined)
- Indicates duplicate global definition
- Check motor_control.cpp for multiple `rover_cmd` definitions
- Solution: Ensure `extern` in .h matches single definition in .cpp

---

## Reference Points

**Modified files:**
- `/home/yupi/almondmatcha/mros2-mbed-chassis-dynamics/workspace/chassis_controller/app.cpp`
- NEW: `motor_control.h`, `motor_control.cpp`
- NEW: `led_status.h`, `led_status.cpp`

**Session log:**
- `/home/yupi/almondmatcha/WORK_SESSION_2025-11-04.md`

**Previous sessions:**
- WORK_SESSION_2025-11-03.md (sensors node 3-task design)
- WORK_SESSION_2025-11-01.md (initial STM32 work)

---

**Estimated time:** 30–45 min (build + verification)
