from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
boot_time = time.time()

while True:
    usec = int((time.time() - boot_time) * 1e6)
    mav.mav.vision_position_estimate_send(
        0,          # let ArduPilot timestamp
        2.0, 2.0, 0.0,   # x,y fixed; z=0 because EKF takes baro
        0.0, 0.0, 0.0
    )

    time.sleep(0.05)  # 20 Hz
