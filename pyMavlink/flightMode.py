from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
CONNECTION = 'udp:127.0.0.1:14550'
#the_connection = mavutil.mavlink_connection(CONNECTION)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (the_connection.target_system, the_connection.target_component))


mode = 'GUIDED'  

if mode not in the_connection.mode_mapping():
    print('Unknown mode:', mode)
    print('Available modes:', list(the_connection.mode_mapping().keys()))
else:
    mode_id = the_connection.mode_mapping()[mode]
    the_connection.mav.set_mode_send(
        the_connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
