
SECONDS_IN_MINUTE = 60

def rpm_to_rps(rpm: float):
    return rpm / SECONDS_IN_MINUTE

def rps_to_rpm(rps: float):
    return rps * SECONDS_IN_MINUTE