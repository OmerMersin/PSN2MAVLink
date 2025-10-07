from pymavlink import mavutil
import time

mav = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
boot_time = time.time()

while True:
    usec = int((time.time() - boot_time) * 1e6)
    mav.mav.vision_position_estimate_send(
        usec,
        10.0, 20.0, -10.5,   # x,y,z in meters (NED)
        0.0, 0.0, 0.0,    # roll, pitch, yaw
        [0]*21            # covariance (optional, pass None if you don’t want it)
    )
    time.sleep(0.05)  # 20 Hz
