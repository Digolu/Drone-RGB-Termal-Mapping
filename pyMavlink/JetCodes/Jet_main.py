from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import Jet_globalFunc as j
import Jet_cameraHeliDetect as hd

j.keyboard_loop()

if __name__ == '__main__':

    threading.Thread(target=j.reader_loop, daemon=True).start()
    threading.Thread(target=j.monitor_mode_changes, daemon=True).start()
    threading.Thread(target=hd.capturar_e_processar, daemon=True).start()

    print(f"A iniciar stream em http://0.0.0.0:{hd.PORT}")
    hd.app.run(host="0.0.0.0", port=hd.PORT, threaded=True)
    