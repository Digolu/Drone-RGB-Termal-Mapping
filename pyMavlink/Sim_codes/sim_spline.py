from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import sim_globalFunc as gf


def catmull_rom_point(p0, p1, p2, p3, t):
    """Interpola um ponto entre p1 e p2 usando Catmull-Rom, t em [0,1]."""
    t2 = t * t
    t3 = t2 * t
    def interp(c0, c1, c2, c3):
        return 0.5 * (
            2*c1 +
            (-c0 + c2) * t +
            (2*c0 - 5*c1 + 4*c2 - c3) * t2 +
            (-c0 + 3*c1 - 3*c2 + c3) * t3
        )
    lat = interp(p0[0], p1[0], p2[0], p3[0])
    lon = interp(p0[1], p1[1], p2[1], p3[1])
    alt = interp(p0[2], p1[2], p2[2], p3[2])
    return (lat, lon, alt)

def generate_spline_path(waypoints, points_per_segment=10):
    """Gera pontos intermédios suavizados entre os waypoints."""
    pts = [waypoints[0]] + waypoints + [waypoints[-1]]  # duplica extremos
    path = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i-1], pts[i], pts[i+1], pts[i+2]
        for step in range(points_per_segment):
            t = step / points_per_segment
            path.append(catmull_rom_point(p0, p1, p2, p3, t))
    path.append(waypoints[-1])
    return path

def wait_close_enough(lat, lon, alt, radius=4.0, alt_tol=1.5, timeout=15):
    """Avança assim que estiver dentro do raio, SEM exigir estabilização (hold_time=0)."""
    start = time.time()
    while time.time() - start < timeout:
        with gf.position_lock:
            lat_now = gf.current_position["lat"]
            lon_now = gf.current_position["lon"]
            alt_now = gf.current_position["alt"]
        if lat_now is not None:
            dist = gf.horizontal_distance(lat_now, lon_now, lat, lon)
            dalt = abs(alt_now - alt)
            if dist <= radius and dalt <= alt_tol:
                return True
        time.sleep(0.1)
    return False

def spline_route_local(waypoints):
    #print("A mudar para GUIDED...")
    #set_mode('GUIDED')
    path = generate_spline_path(waypoints, points_per_segment=8)
    for lat, lon, alt in path:
        gf.drone.mav.set_position_target_global_int_send(
            0, gf.drone.target_system, gf.drone.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000_1111_1111_1000,
            int(lat * 1e7), int(lon * 1e7), alt,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        # raio maior => avança antes de desacelerar totalmente
        wait_close_enough(lat, lon, alt, radius=6.0, alt_tol=1.5, timeout=15)
    print("Rota spline (local) completa!")
