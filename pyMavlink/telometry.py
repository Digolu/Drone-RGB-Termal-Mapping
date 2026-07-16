from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (the_connection.target_system, the_connection.target_component))

while True:
        
    msg = the_connection.recv_match(blocking=True)
    if msg is None:
        continue

    msg_type = msg.get_type()



    if msg_type == "ATTITUDE":
        print(f"Roll: {msg.roll:.2f} Pitch: {msg.pitch:.2f} Yaw: {msg.yaw:.2f}")

    elif msg_type == "GLOBAL_POSITION_INT":
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        print(f"Lat: {lat:.6f} Lon: {lon:.6f} Alt: {alt:.1f}m")

    elif msg_type == "VFR_HUD":
        print(f"Groundspeed: {msg.groundspeed:.1f} Heading: {msg.heading}")

    elif msg_type == "SYS_STATUS":
        battery_v = msg.voltage_battery / 1000.0
        print(f"Battery: {battery_v:.2f}V Current: {msg.current_battery/100:.2f}A")

    elif msg_type == "HEARTBEAT":
        print("Heartbeat recebido")
        print('='*40) 
