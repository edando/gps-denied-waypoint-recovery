# GPS-Denied Waypoint Recovery System

A GPS-denied navigation failover system built on ArduPilot SITL using EKF3 dual-lane architecture and optical flow sensors. The system detects GPS jamming in real time via MAVLink flag transitions, maintains controlled flight on backup sensors, and produces a georeferenced KML intelligence product with flight track segmentation and denial zone overlay.

Built and validated across five flight runs as part of a tactical UAS platform research project.

---

## Architecture

```
ArduPilot SITL (ArduCopter V4.8.0-dev)
        │
        ├── UDP :14551 ──► Mission Planner        (flight monitoring)
        ├── UDP :14552 ──► phase3_gps_denial.py   (GPS jam injection)
        └── UDP :14553 ──► phase4_ew_logger.py    (EW event detection)
                                   │
                                   └──► ew_event_log.kml
                                        (georeferenced KML output)
```

### EKF3 Lane Architecture

|                | Lane 0 — GPS        | Lane 1 — GPS-Denied        |
|----------------|---------------------|----------------------------|
| Position       | GPS                 | None (dead reckoning)      |
| Velocity       | GPS                 | Optical Flow               |
| Altitude       | Barometer           | Rangefinder                |
| Active when    | GPS healthy         | GPS jammed or degraded     |

Lane 1 carries no absolute position source. Optical flow provides velocity only — the EKF dead reckons from the last known GPS fix and drift accumulates over time. On-course flight post-denial lasted approximately 28 seconds. Integrating a visual-inertial odometry source such as the Intel RealSense T265 is the natural next step.

---

## Requirements

- Python 3.8+
- pymavlink — `pip install pymavlink`
- ArduPilot SITL — ArduCopter V4.8.0-dev or later
- Google Earth Pro — for KML output visualisation

---

## SITL Launch Command

```bash
sim_vehicle.py -v ArduCopter --console --map \
  --out udp:127.0.0.1:14551 \
  --out udp:127.0.0.1:14552 \
  --out udp:127.0.0.1:14553
```

Wait for `APM: EKF3 IMU0 is using GPS` in the console before proceeding.

---

## How to Run

1. Start SITL using the launch command above
2. Start the EW logger — **must be running before the mission script**
```bash
source venv-ardupilot/bin/activate
python3 phase4_ew_logger.py
```
3. Start the GPS denial mission script
```bash
python3 phase3_gps_denial.py
```
4. Press `Ctrl+C` on the logger when the flight is complete — KML is written to `~/ew_event_log.kml`
5. Open the KML in Google Earth Pro

---

## Key Parameters

The following parameters were changed from ArduPilot defaults during Phase 5 tuning. Full documentation in `docs/parameter_config_guide.docx`.

| Parameter | Default | Phase 5 Value | Reason |
|---|---|---|---|
| `EK3_FLOW_M_NSE` | 0.25 | 0.5 | Reduce EKF trust in optical flow to suppress position divergence spikes |
| `EK3_FLOW_I_GATE` | 300 | 200 | Tighten innovation gate to reject outlier flow measurements |
| `EK3_RNG_USE_HGT` | -1 | 70 | Enable rangefinder altitude blending below 35m AGL — eliminated altitude collapse |
| `EK3_ALT_M_NSE` | 2 | 1 | Tighten barometric altitude trust |
| `EK3_ERR_THRESH` | 0.2 | 0.3 | Reduce sensitivity to transient GPS flickers causing false recovery events |
| `FS_EKF_THRESH` | 0.8 | 2.0 | Increase failsafe headroom to tolerate optical flow variance spikes |
| `EK3_MAG_CAL` | 3 | 0 | Freeze magnetometer calibration — prevents yaw drift during denial |
| `EK3_MAG_MASK` | 0 | 3 | Force both EKF lanes to share fixed mag reference — prevents yaw divergence between lanes |
| `WP_YAW_BEHAVIOR` | 2 | 0 | Disable waypoint yaw — prevents autopilot fighting heading lock at denial time |

Load the validated parameter set directly:
```
param load param_phase_5.param
```

---

## Results

| Result | Value |
|---|---|
| Reduction in EKF position divergence through tuning | 98% |
| Altitude precision during GPS denial | ±4.5cm std dev |
| EKF position spikes in final tuned run | 0 |
| Flight runs across iterative tuning cycle | 5 |

---

## Repository Structure

```
gps-denied-waypoint-recovery/
├── phase3_gps_denial.py             # GPS jamming injection script
├── phase4_ew_logger.py              # Real-time EW event logger
├── param_setup.param                # Validated ArduPilot parameter set
├── ew_event_log.kml                 # KML output from final validated run
├── EW_Event_Logger_User_Manual.docx # EW logger operational reference
├── GPS_Denied_Parameter_Config_G... # Full parameter documentation
├── .gitignore
└── README.md

---

## Documentation

- [Parameter Configuration Guide](docs/parameter_config_guide.docx) — all relevant ArduPilot parameters, tuning rationale, and before/after results
- [EW Event Logger User Manual](docs/ew_logger_user_manual.docx) — complete operational reference for phase4_ew_logger.py

---

## Phase Roadmap

| Phase | Description | Status |
|---|---|---|
| 1 | Environment setup and SITL configuration | ✅ Complete |
| 2 | EKF3 dual-lane architecture validation | ✅ Complete |
| 3 | GPS denial injection and lane switch validation | ✅ Complete |
| 4 | EW Event Logger build and KML output validation | ✅ Complete |
| 5 | Integration testing, tuning, and documentation | ✅ Complete |
| 6 | Intel RealSense T265 VIO integration for Lane 1 absolute position | Proposed |
