# gps_health.py
# Module to monitor GPS health parameters for a rover or drone

from datetime import datetime, timedelta
import math

class GPSHealthStatus:
    """
    Container for GPS health check results.
    Attributes:
        fix_ok (bool): True if GPS fix type meets minimum requirement.
        satellites_ok (bool): True if number of satellites ≥ threshold.
        dop_ok (bool): True if HDOP and/or PDOP below thresholds.
        update_rate_ok (bool): True if time between fixes ≤ threshold.
        smooth_ok (bool): True if position jump ≤ threshold.
        time_ok (bool): True if GPS time valid.
        messages (list of str): Warnings or failures.
    """
    def __init__(self, fix_ok, satellites_ok, dop_ok,
                 update_rate_ok, smooth_ok, time_ok, messages):
        self.fix_ok = fix_ok
        self.satellites_ok = satellites_ok
        self.dop_ok = dop_ok
        self.update_rate_ok = update_rate_ok
        self.smooth_ok = smooth_ok
        self.time_ok = time_ok
        self.messages = messages

    def is_healthy(self):
        """
        Return True if all checks pass.
        """
        return all([self.fix_ok,
                    self.satellites_ok,
                    self.dop_ok,
                    self.update_rate_ok,
                    self.smooth_ok,
                    self.time_ok])


def haversine_distance(pos1, pos2):
    """
    Compute the great-circle distance between two (lat, lon) tuples in meters.
    """
    R = 6371000.0  # Earth radius in meters
    lat1, lon1 = map(math.radians, pos1)
    lat2, lon2 = map(math.radians, pos2)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    return R * c


def check_gps_health(fix_type,
                     num_satellites,
                     hdop,
                     pdop,
                     timestamp,
                     last_timestamp=None,
                     last_position=None,
                     current_position=None,
                     thresholds=None):
    """
    Check various GPS health parameters and return a GPSHealthStatus.

    Args:
        fix_type (int): GPS fix type (0=no fix,1=GPS,2=DGPS,4=RTK fixed,5=RTK float).
        num_satellites (int): Number of satellites used.
        hdop (float): Horizontal dilution of precision.
        pdop (float): Position dilution of precision.
        timestamp (datetime): Timestamp of current GPS fix.
        last_timestamp (datetime, optional): Timestamp of previous fix.
        last_position (tuple(lat, lon), optional): Previous GPS coordinates.
        current_position (tuple(lat, lon), optional): Current GPS coordinates.
        thresholds (dict, optional): Override default thresholds.

    Returns:
        GPSHealthStatus: Object summarizing pass/fail and messages.
    """
    # Default thresholds
    default = {
        'min_fix_type': 1,
        'min_satellites': 6,
        'max_hdop': 3.0,
        'max_pdop': 3.0,
        'max_interval': 2.0,      # seconds
        'max_jump': 5.0           # meters
    }
    if thresholds:
        default.update(thresholds)

    msgs = []

    # 1. Fix status
    fix_ok = fix_type >= default['min_fix_type']
    if not fix_ok:
        msgs.append(f"GPS fix type too low ({fix_type}); needs ≥ {default['min_fix_type']}")

    # 2. Satellites
    satellites_ok = num_satellites >= default['min_satellites']
    if not satellites_ok:
        msgs.append(f"Only {num_satellites} satellites; needs ≥ {default['min_satellites']}")

    # 3. DOP
    dop_ok = (hdop <= default['max_hdop']) and (pdop <= default['max_pdop'])
    if not dop_ok:
        msgs.append(f"High DOP values (HDOP={hdop:.2f}, PDOP={pdop:.2f}); max allowed {default['max_hdop']}/{default['max_pdop']}")

    # 4. Update rate
    update_rate_ok = True
    if last_timestamp:
        interval = (timestamp - last_timestamp).total_seconds()
        update_rate_ok = interval <= default['max_interval']
        if not update_rate_ok:
            msgs.append(f"GPS update stalled: interval {interval:.2f}s > {default['max_interval']}s")

    # 5. Position jump (if positions provided)
    smooth_ok = True
    if last_position and current_position:
        dist = haversine_distance(last_position, current_position)
        smooth_ok = dist <= default['max_jump']
        if not smooth_ok:
            msgs.append(f"GPS jump detected: moved {dist:.1f}m > {default['max_jump']}m")

    # 6. Time validity
    time_ok = isinstance(timestamp, datetime)
    if not time_ok:
        msgs.append("Invalid GPS timestamp")

    status = GPSHealthStatus(
        fix_ok=fix_ok,
        satellites_ok=satellites_ok,
        dop_ok=dop_ok,
        update_rate_ok=update_rate_ok,
        smooth_ok=smooth_ok,
        time_ok=time_ok,
        messages=msgs
    )
    return status
