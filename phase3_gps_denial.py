from pymavlink import mavutil
import time

connection = mavutil.mavlink_connection('udp:0.0.0.0:14552')
connection.wait_heartbeat()
print("Heartbeat confirmed")
print("Waiting 25 seconds before GPS denial...")
time.sleep(25)

# GPS Denial via SIM_GPS1_ENABLE
connection.mav.param_set_send(
    connection.target_system,
    connection.target_component,
    b'SIM_GPS1_JAM',
    1,
    mavutil.mavlink.MAV_PARAM_TYPE_INT8
)
print("GPS denied via SIM_GPS1_JAM - optical flow takeover")

# Lock heading
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_CONDITION_YAW,
    0, 0, 10, 1, 1, 0, 0, 0
)
print("Heading locked at moment of GPS denial")

# Monitor variance and trigger RTL
print("Monitoring position variance - RTL triggers if drift exceeds 10m...")
DRIFT_THRESHOLD = 10.0
max_variance = 0.0
rtl_triggered = False
variance_readings = []
denial_time = time.time()

while True:
    alt_msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if alt_msg:
        altitude = alt_msg.relative_alt / 1000.0
        if altitude < 0.5 and rtl_triggered:
            elapsed = time.time() - denial_time
            print("\n========== PHASE 3.7 FLIGHT REPORT ==========")
            print(f"GPS denial duration:     {elapsed:.1f} seconds")
            print(f"Peak position variance:  {max_variance:.2f}m")
            print(f"RTL triggered:           {'Yes' if rtl_triggered else 'No'}")
            print(f"Total variance readings: {len(variance_readings)}")
            print(f"Average variance:        {sum(variance_readings)/len(variance_readings):.2f}m" if variance_readings else "N/A")
            print("==============================================")
            break

    msg = connection.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
    if msg:
        pos_variance = msg.pos_horiz_variance
        variance_readings.append(pos_variance)
        if pos_variance > max_variance:
            max_variance = pos_variance
        if not rtl_triggered:
            print(f"Current position variance: {pos_variance:.2f}m")
        if pos_variance > DRIFT_THRESHOLD and not rtl_triggered:
            print(f"Drift threshold exceeded: {pos_variance:.2f}m - triggering RTL")
            connection.mav.command_long_send(
                connection.target_system,
                connection.target_component,
                mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            print("RTL triggered - returning to base")
            rtl_triggered = True
