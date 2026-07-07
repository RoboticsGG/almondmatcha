# STM32 Firmware Changes Summary

**Branch:** `multi-domain` (baseline tri-domain) | **Base:** mROS-base/mros2 v0.5.4  
**Boards:** NUCLEO-F767ZI × 2 — chassis (192.168.1.2) and sensors (192.168.1.6)

Narrative investigation: [BUG_INVESTIGATION_LOG.md](BUG_INVESTIGATION_LOG.md)  
Full patch analysis: [EMBEDDEDRTPS_PATCHES.md](EMBEDDEDRTPS_PATCHES.md)

---

## 1. embeddedRTPS Patches (5 patches)

Applied by `build.bash` from `platform/patches/`. Idempotent — safe to rebuild.

| Patch | Root cause fixed |
|-------|-----------------|
| `001-rtps-deserialization-fixes.patch` | `octetsToNextHeader=0` misparse (HardFault); AckNack bitmap overflow; TopicData name overflow |
| `002-rtps-hardfault-prevention.patch` | Recv buffer isolation from pbuf; guard-word corruption detection; F767ZI pointer validation |
| `003-discovery-pool-diagnostic.patch` | MemoryPool error adds `(size=N, used=N)`; SEDP_VERBOSE=0 silences UART spam |
| `004-spdp-pbuf-and-sedp-locator-fallback.patch` | `PBUF_TRANSPORT`→`PBUF_RAW`; SEDP locator 3-level fallback; SEDP immediate resend |
| `005-multicast-address-detection-fix.patch` | `isMulticastAddress()` byte-order bug; `readLocatorIntoList()` operator precedence bug |

---

## 2. `platform/rtps/config.h` Tuning

Multi-domain (baseline): **11 D5 participants** (D6 vision and D4 telemetry nodes are invisible to STM32).

| Parameter | Default | Production | Reason |
|-----------|---------|------------|--------|
| `MAX_NUM_PARTICIPANTS` | 15 | **1** | Pool of 15 = 525 KB BSS → OOM at boot |
| `SPDP_MAX_NUMBER_FOUND_PARTICIPANTS` | 14 | **30** | 14 exhausted by daemons + CLI tools |
| `NUM_WRITER_PROXIES_PER_READER` | 6 | **30** | 6 < 11 D5 nodes → MemoryPool panic |
| `NUM_READER_PROXIES_PER_WRITER` | 6 | **30** | Same |
| `THREAD_POOL_READER_STACKSIZE` | 4096 | **8192** | 4096 → HardFault at 99.6% stack depth |
| `THREAD_POOL_WORKLOAD_QUEUE_LENGTH` | 20 | **40** | 20 → drops during 3-node SPDP burst |
| `SF_WRITER_HB_PERIOD_MS` | 2000 | **1000** | Faster SEDP reliability |
| `HISTORY_SIZE_STATEFUL` | 64 | **5** | Not needed; saves BSS |
| `NUM_WRITERS_PER_PARTICIPANT` | varies | **16** | 10 actual publishers + margin |
| `NUM_READERS_PER_PARTICIPANT` | varies | **16** | 11 actual subscribers + margin |

Full reference: [RTPS_CONFIG_REFERENCE.md](RTPS_CONFIG_REFERENCE.md)

---

## 3. `build.bash` Fixes

| Fix | Before | After |
|-----|--------|-------|
| mbed-os acquisition | `mbed-tools deploy` (broken in Docker) | `git clone --depth 1 mbed-os-6.17.0` |
| mros2 acquisition | `mbed-tools deploy` | `git clone --depth 1 mros2-mbed` |
| embeddedRTPS submodule | Not initialized | `git -C mros2 submodule update --init` |
| Patch application | None | Loop over `platform/patches/*.patch` with `--reverse` idempotency check |
| Docker safe.directory | `git: dubious ownership` | `git config --global --add safe.directory '*'` |
| msgs_ifaces sync | Manual | Auto `rsync` from `common_ifaces/msgs_ifaces/` |
| File ownership | Root-owned after Docker | `chown -R $SUDO_USER build/ mros2/ mbed-os/ platform/` |

Chassis only: `sed` rename of `chassis_im_u.hpp` → `chassis_imu.hpp` typo in `platform/templates.hpp`.

---

## 4. Application Firmware Changes

| Change | Board | Detail |
|--------|-------|--------|
| Multi-line boot banner | Both | Prints board IP, domain, SPDP_MAX, node name, pub/sub topics |
| Non-blocking discovery wait | Both | 3s wait inside task; spin() runs during wait → responds to SPDP from t=0 |
| Print-once sensor output | Both | _first_print_done flag — was flooding at 100 Hz |
| GNSS task rate | Sensors | 10 Hz → 2 Hz (RTK fixes arrive at ~1 Hz) |
| Node name | Sensors | mros2_node_sensors_d6 → mros2_node_sensors |
| IMU reader stack | Chassis | 2048 → 4096 bytes (CDR + embeddedRTPS + lwIP UDP send) |
| Memory reporter | Both | memory_reporter.h — JSON {"type":"STM32_STATS",...} every 1s for POC collection |
| Serial mutex fix | Both | mr_serial_mutex() function-local static — fixes per-TU copy when gnss_reader.cpp includes header |
| Heartbeat rate | Both | Serial output reduced to 2 Hz |

---

## 5. lwIP Configuration (`mbed_app.json`)

`lwip.pbuf-pool-size: 32` — default 16 exhausted during 30+ simultaneous DDS discovery packets.

---

## 6. FastDDS Interface Selection

`fastdds_base.xml` / `fastdds_rover.xml` pin DDS transport to the rover Ethernet interface. Without this, FastDDS defaults to WiFi and STM32 SPDP packets are never received.

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/almondmatcha/ws_base/fastdds_base.xml
ros2 daemon stop && ros2 daemon start
```
