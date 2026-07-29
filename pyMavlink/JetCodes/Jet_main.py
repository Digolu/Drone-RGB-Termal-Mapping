from pymavlink import mavutil
import math
import sys, tty, termios, threading, time, queue
import Jet_globalFunc as j


if __name__ == '__main__':
    threading.Thread(target=j.reader_loop, daemon=True).start()
    threading.Thread(target=j.monitor_mode_changes, daemon=True).start()
    j.keyboard_loop()