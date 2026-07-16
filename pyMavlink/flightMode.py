from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (the_connection.target_system, the_connection.target_component))


mode = 'ALT_HOLD'  

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
