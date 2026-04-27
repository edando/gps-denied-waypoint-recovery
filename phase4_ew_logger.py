#!/usr/bin/env python3
"""
phase4_ew_logger.py — Phase 4: EW Event Logger
Task 4.6: Flight track visualization with color coding
"""

import math
import time
from datetime import datetime, timezone
from pymavlink import mavutil

# ── Config ────────────────────────────────────────────────────────────────────
CONNECTION        = "udp:127.0.0.1:14553"
HEARTBEAT_TIMEOUT = 30
DEBOUNCE_SECS     = 3.0
FLAGS_LANE1       = 33663
FLAGS_LANE1_B     = 33135
FLAGS_LANE0_A     = 831
FLAGS_LANE0_B     = 895
DENIAL_RADIUS_M   = 50.0
POLYGON_POINTS    = 8
KML_OUTPUT        = "/home/raybond/ew_event_log.kml"

# ── Shared state ──────────────────────────────────────────────────────────────
events    = []
track_log = []   # list of (epoch, lat, lon, alt_rel)

position_cache = {
    "lat": None, "lon": None, "alt": None,
    "relative_alt": None, "heading": None, "updated_at": None,
}

# ── Connection ────────────────────────────────────────────────────────────────
def connect(connection_string: str) -> mavutil.mavfile:
    print(f"[INIT] Connecting to SITL on {connection_string} ...")
    mav = mavutil.mavlink_connection(connection_string)
    print("[INIT] Waiting for heartbeat ...")
    msg = mav.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if msg is None:
        raise TimeoutError(f"No heartbeat within {HEARTBEAT_TIMEOUT}s")
    print(f"[INIT] Heartbeat OK — system {mav.target_system}, "
          f"component {mav.target_component}\n")
    return mav


# ── Lane classification ───────────────────────────────────────────────────────
def is_lane1(flags: int) -> bool:
    return flags in (FLAGS_LANE1, FLAGS_LANE1_B)

def is_lane0(flags: int) -> bool:
    return flags in (FLAGS_LANE0_A, FLAGS_LANE0_B)


# ── Position cache + track logging ───────────────────────────────────────────
def update_position(msg) -> None:
    position_cache["lat"]          = msg.lat          / 1e7
    position_cache["lon"]          = msg.lon          / 1e7
    position_cache["alt"]          = msg.alt          / 1000.0
    position_cache["relative_alt"] = msg.relative_alt / 1000.0
    position_cache["heading"]      = msg.hdg          / 100.0
    position_cache["updated_at"]   = time.time()

    # Append to track log
    track_log.append((
        time.time(),
        position_cache["lat"],
        position_cache["lon"],
        position_cache["relative_alt"],
    ))

def get_position_snapshot() -> dict:
    return dict(position_cache)


# ── Polygon estimation ────────────────────────────────────────────────────────
EARTH_RADIUS_M = 6_371_000.0

def offset_latlon(lat, lon, bearing_deg, distance_m):
    bearing = math.radians(bearing_deg)
    d_lat   = (distance_m * math.cos(bearing)) / EARTH_RADIUS_M
    d_lon   = (distance_m * math.sin(bearing)) / \
              (EARTH_RADIUS_M * math.cos(math.radians(lat)))
    return lat + math.degrees(d_lat), lon + math.degrees(d_lon)

def haversine_m(lat1, lon1, lat2, lon2) -> float:
    r = EARTH_RADIUS_M
    p = math.pi / 180
    a = (math.sin((lat2 - lat1) * p / 2) ** 2 +
         math.cos(lat1 * p) * math.cos(lat2 * p) *
         math.sin((lon2 - lon1) * p / 2) ** 2)
    return 2 * r * math.asin(math.sqrt(a))

def estimate_denial_polygon(events, radius_m=DENIAL_RADIUS_M,
                             n_points=POLYGON_POINTS):
    valid  = [e for e in events if e.get("pos_valid")]
    onset    = next((e for e in valid if e["type"] == "DENIAL_ONSET"), None)
    recovery = next((e for e in valid if e["type"] == "GPS_RECOVERED"), None)

    if not onset:
        print("[POLYGON] No valid DENIAL_ONSET event.")
        return []

    if not recovery:
        c_lat, c_lon = onset["lat"], onset["lon"]
        print(f"[POLYGON] Circular r={radius_m}m @ ({c_lat:.7f}, {c_lon:.7f})")
        verts = []
        for i in range(n_points):
            v_lat, v_lon = offset_latlon(c_lat, c_lon,
                                         360.0 * i / n_points, radius_m)
            verts.append((v_lat, v_lon))
        verts.append(verts[0])
        return verts

    lat1, lon1 = onset["lat"],    onset["lon"]
    lat2, lon2 = recovery["lat"], recovery["lon"]
    c_lat = (lat1 + lat2) / 2
    c_lon = (lon1 + lon2) / 2
    sep   = haversine_m(lat1, lon1, lat2, lon2)
    semi_major = (sep / 2) + radius_m
    semi_minor = radius_m
    bearing    = math.degrees(math.atan2(lon2 - lon1, lat2 - lat1)) % 360

    print(f"[POLYGON] Elliptical — sep={sep:.1f}m bearing={bearing:.1f}°")
    verts = []
    for i in range(n_points):
        angle = 2 * math.pi * i / n_points
        lx    = semi_major * math.cos(angle)
        ly    = semi_minor * math.sin(angle)
        b_rad = math.radians(bearing)
        rx    = lx * math.cos(b_rad) - ly * math.sin(b_rad)
        ry    = lx * math.sin(b_rad) + ly * math.cos(b_rad)
        v_lat = c_lat + math.degrees(rx / EARTH_RADIUS_M)
        v_lon = c_lon + math.degrees(
            ry / (EARTH_RADIUS_M * math.cos(math.radians(c_lat))))
        verts.append((v_lat, v_lon))
    verts.append(verts[0])
    return verts


# ── Track splitting ───────────────────────────────────────────────────────────
def split_track(track: list, events: list):
    """
    Split track_log into GPS and denied segments based on event epochs.
    Returns (gps_track, denied_track) — each a list of (lat, lon, alt_rel).
    """
    onset    = next((e for e in events if e["type"] == "DENIAL_ONSET"), None)
    recovery = next((e for e in events if e["type"] == "GPS_RECOVERED"), None)

    if not onset:
        return track, []

    onset_epoch    = onset["epoch"]
    recovery_epoch = recovery["epoch"] if recovery else float("inf")

    gps_track    = [(la, lo, al) for t, la, lo, al in track
                    if t <= onset_epoch]
    denied_track = [(la, lo, al) for t, la, lo, al in track
                    if onset_epoch <= t <= recovery_epoch]
    return gps_track, denied_track


# ── KML generation ────────────────────────────────────────────────────────────
def build_kml(events: list, polygon: list,
              gps_track: list, denied_track: list,
              output_path: str) -> None:

    def coord(lat, lon, alt=0):
        return f"{lon},{lat},{alt}"

    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append('<kml xmlns="http://www.opengis.net/kml/2.2">')
    lines.append('  <Document>')
    lines.append('    <name>EW Event Log</name>')
    lines.append('    <description>GPS Denial Zone — ArduPilot SITL Phase 4</description>')

    # ── Styles ────────────────────────────────────────────────────────────────
    styles = [
        ("denial",      "ff0000ff", None),    # red marker
        ("recovery",    "ff00ff00", None),    # green marker
    ]
    for style_id, colour, _ in styles:
        lines.append(f'    <Style id="{style_id}">')
        lines.append(f'      <IconStyle>')
        lines.append(f'        <color>{colour}</color>')
        lines.append(f'        <scale>1.2</scale>')
        lines.append(f'        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>')
        lines.append(f'      </IconStyle>')
        lines.append(f'    </Style>')

    # Polygon style
    lines.append('    <Style id="denial_zone">')
    lines.append('      <LineStyle><color>ff0000ff</color><width>2</width></LineStyle>')
    lines.append('      <PolyStyle><color>660000ff</color><fill>1</fill><outline>1</outline></PolyStyle>')
    lines.append('    </Style>')

    # GPS track style — green line
    lines.append('    <Style id="gps_track">')
    lines.append('      <LineStyle><color>ff00ff00</color><width>3</width></LineStyle>')
    lines.append('      <PolyStyle><fill>0</fill></PolyStyle>')
    lines.append('    </Style>')

    # Denied track style — red line
    lines.append('    <Style id="denied_track">')
    lines.append('      <LineStyle><color>ff0000ff</color><width>3</width></LineStyle>')
    lines.append('      <PolyStyle><fill>0</fill></PolyStyle>')
    lines.append('    </Style>')

    # ── Event placemarks ──────────────────────────────────────────────────────
    for e in events:
        if not e.get("pos_valid"):
            continue
        style = "denial" if e["type"] == "DENIAL_ONSET" else "recovery"
        name  = "GPS DENIAL ONSET" if e["type"] == "DENIAL_ONSET" \
                else "GPS RECOVERED"
        alt   = e.get("alt_rel") or 0.0
        desc  = (
            f"Time: {e['timestamp']}&#10;"
            f"EKF flags: {e['flags']}&#10;"
            f"pos_horiz_var: {e['pos_horiz_var']:.4f}&#10;"
            f"vel_var: {e['vel_var']:.4f}&#10;"
            f"oscillations: {e['oscillations']}&#10;"
            f"heading: {e['heading']:.1f}°&#10;"
            f"alt AMSL: {e['alt_amsl']:.1f}m"
        )
        lines.append('    <Placemark>')
        lines.append(f'      <name>{name}</name>')
        lines.append(f'      <description>{desc}</description>')
        lines.append(f'      <styleUrl>#{style}</styleUrl>')
        lines.append('      <Point>')
        lines.append('        <altitudeMode>relativeToGround</altitudeMode>')
        lines.append(f'        <coordinates>{coord(e["lat"], e["lon"], alt)}</coordinates>')
        lines.append('      </Point>')
        lines.append('    </Placemark>')

    # ── Denial zone polygon ───────────────────────────────────────────────────
    if polygon:
        poly_coords = "\n          ".join(
            coord(lat, lon, 0) for lat, lon in polygon
        )
        lines.append('    <Placemark>')
        lines.append('      <name>GPS Denial Zone</name>')
        lines.append('      <styleUrl>#denial_zone</styleUrl>')
        lines.append('      <Polygon>')
        lines.append('        <altitudeMode>clampToGround</altitudeMode>')
        lines.append('        <outerBoundaryIs><LinearRing><coordinates>')
        lines.append(f'          {poly_coords}')
        lines.append('        </coordinates></LinearRing></outerBoundaryIs>')
        lines.append('      </Polygon>')
        lines.append('    </Placemark>')

    # ── GPS track segment — green ─────────────────────────────────────────────
    if gps_track:
        gps_coords = "\n          ".join(
            coord(la, lo, al) for la, lo, al in gps_track
        )
        lines.append('    <Placemark>')
        lines.append('      <name>GPS Track (Lane 0)</name>')
        lines.append('      <styleUrl>#gps_track</styleUrl>')
        lines.append('      <LineString>')
        lines.append('        <altitudeMode>relativeToGround</altitudeMode>')
        lines.append('        <coordinates>')
        lines.append(f'          {gps_coords}')
        lines.append('        </coordinates>')
        lines.append('      </LineString>')
        lines.append('    </Placemark>')

    # ── Denied track segment — red ────────────────────────────────────────────
    if denied_track:
        denied_coords = "\n          ".join(
            coord(la, lo, al) for la, lo, al in denied_track
        )
        lines.append('    <Placemark>')
        lines.append('      <name>Denied Track (Lane 1)</name>')
        lines.append('      <styleUrl>#denied_track</styleUrl>')
        lines.append('      <LineString>')
        lines.append('        <altitudeMode>relativeToGround</altitudeMode>')
        lines.append('        <coordinates>')
        lines.append(f'          {denied_coords}')
        lines.append('        </coordinates>')
        lines.append('      </LineString>')
        lines.append('    </Placemark>')

    lines.append('  </Document>')
    lines.append('</kml>')

    with open(output_path, "w") as f:
        f.write("\n".join(lines))

    print(f"[KML] Written to {output_path}")
    print(f"[KML] GPS track points:    {len(gps_track)}")
    print(f"[KML] Denied track points: {len(denied_track)}")


# ── Main detection loop ───────────────────────────────────────────────────────
def detect_lane_switches(mav: mavutil.mavfile) -> list:
    print("[LISTEN] Receiving all MAVLink messages, routing by type")
    print(f"         Debounce window: {DEBOUNCE_SECS}s\n")
    print(f"{'Timestamp':<26} {'flags':>6} {'vel_var':>8} "
          f"{'pos_h_var':>10} {'pos_v_var':>10} {'cmp_var':>8}  status")
    print("-" * 90)

    prev_flags        = None
    pending_lane      = None
    pending_since     = None
    current_lane      = 0
    oscillation_count = 0

    while True:
        raw = mav.recv_match(blocking=True, timeout=5)
        if raw is None:
            print("[WARN] No messages in 5s")
            continue

        msg_type = raw.get_type()

        if msg_type == "GLOBAL_POSITION_INT":
            update_position(raw)
            continue

        if msg_type != "EKF_STATUS_REPORT":
            continue

        msg    = raw
        now    = time.time()
        flags  = msg.flags
        ts     = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3]
        status = ""

        if prev_flags is not None and flags != prev_flags:
            new_lane = 1 if is_lane1(flags) else 0 if is_lane0(flags) else -1
            if new_lane != -1 and new_lane != current_lane:
                if pending_lane is None:
                    pending_lane  = new_lane
                    pending_since = now
                    status = f"[PENDING → Lane {new_lane}]"
                elif new_lane != pending_lane:
                    oscillation_count += 1
                    pending_lane  = new_lane
                    pending_since = now
                    status = f"[OSCILLATION #{oscillation_count} → Lane {new_lane}]"

        if pending_lane is not None and (now - pending_since) >= DEBOUNCE_SECS:
            event_type   = "DENIAL_ONSET" if pending_lane == 1 else "GPS_RECOVERED"
            current_lane = pending_lane
            pending_lane = None

            pos       = get_position_snapshot()
            pos_age   = (now - pos["updated_at"]) if pos["updated_at"] else None
            pos_valid = pos_age is not None and pos_age < 2.0

            event = {
                "type":          event_type,
                "timestamp":     ts,
                "epoch":         now,
                "flags":         flags,
                "vel_var":       msg.velocity_variance,
                "pos_horiz_var": msg.pos_horiz_variance,
                "pos_vert_var":  msg.pos_vert_variance,
                "compass_var":   msg.compass_variance,
                "oscillations":  oscillation_count,
                "lat":           pos["lat"],
                "lon":           pos["lon"],
                "alt_amsl":      pos["alt"],
                "alt_rel":       pos["relative_alt"],
                "heading":       pos["heading"],
                "pos_valid":     pos_valid,
                "pos_age_secs":  round(pos_age, 3) if pos_age else None,
            }
            events.append(event)
            status = f"*** CONFIRMED {event_type} ***"

            print(f"\n{'='*90}")
            print(f"  EVENT:      {event_type}")
            print(f"  Time:       {ts}")
            print(f"  EKF:        flags={flags} | pos_horiz_var={msg.pos_horiz_variance:.4f}"
                  f" | oscillations={oscillation_count}")
            if pos_valid:
                print(f"  POSITION:   lat={pos['lat']:.7f}  lon={pos['lon']:.7f}"
                      f"  alt={pos['alt']:.1f}m  hdg={pos['heading']:.1f}°")
                print(f"  pos_age:    {pos_age:.3f}s")
            else:
                print(f"  POSITION:   *** NOT AVAILABLE ***")
            print(f"{'='*90}\n")

        print(
            f"{ts:<26} "
            f"{flags:>6} "
            f"{msg.velocity_variance:>8.4f} "
            f"{msg.pos_horiz_variance:>10.4f} "
            f"{msg.pos_vert_variance:>10.4f} "
            f"{msg.compass_variance:>8.4f}  "
            f"{status}"
        )

        prev_flags = flags

    return events


# ── Entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    try:
        mav    = connect(CONNECTION)
        result = detect_lane_switches(mav)
    except KeyboardInterrupt:
        print(f"\n[EXIT] Listener stopped.")
        print(f"[SUMMARY] Confirmed events: {len(events)}")
        for i, e in enumerate(events, 1):
            print(f"  {i}. {e['type']} @ {e['timestamp']}")
            if e["pos_valid"]:
                print(f"     lat={e['lat']:.7f}  lon={e['lon']:.7f}"
                      f"  alt={e['alt_amsl']:.1f}m  hdg={e['heading']:.1f}°")

        print(f"\n[POLYGON] Estimating denial zone ...")
        polygon = estimate_denial_polygon(events)

        print(f"\n[TRACK] Splitting flight track ...")
        gps_track, denied_track = split_track(track_log, events)
        print(f"[TRACK] GPS points: {len(gps_track)}  "
              f"Denied points: {len(denied_track)}")

        print(f"\n[KML] Generating output file ...")
        build_kml(events, polygon, gps_track, denied_track, KML_OUTPUT)

    except TimeoutError as e:
        print(f"[ERROR] {e}")
