#! /usr/bin/python
import os
import sys
import time
import threading


class WirelessLog(object):
    def __init__(self, filename='wireless.log'):
        self.log_file = filename
        self.last_read = 0
        self.last_write = 0

    def write(self, entry):
        with open(self.log_file, 'a') as log:
            self.last_write = time.time()
            log.write(entry)

    def read(self, lines=10):
        with open(self.log_file, 'r') as log:
            self.last_read = time.time()
            return ''.join(log.readlines()[-10:])

    def display(self, lines=10):
        if self.last_write >= self.last_read:
            os.system('clear')
            print self.read(lines)


class WirelessConnection(object):
    def __init__(self, wait_time=5):
        def __monitor_connection():
            while True:
                con = not os.system('ping -q -w9 -c1 8.8.8.8 >/dev/null 2>/dev/null')
                t = time.time()
                if con != self.con[-1][1]:
                    self.con.append((t, con))
                time.sleep(wait_time)
        con = not os.system('ping -q -w9 -c1 8.8.8.8 >/dev/null 2>/dev/null')
        self.con = [(time.time(), con)]
        self._auto_connect = False
        self._ac_lock = threading.Lock()
        th = threading.Thread(target=self.is_connected)
        th.daemon = True
        th.start()

    def wait_for_change(self, timeout=float('inf')):
        "Returns True when connection state changes or False at timeout."
        t = time.time()
        while (self.con[-1][0] < t and time.time() < t + timeout):
            time.sleep(1)
        return self.con[-1][0] >= t

    def connect(self):
        if not self.is_connected():
            os.system('sudo ifconfig wlan0 down && '
                      'sudo ifconfig wlan0 up && '
                      'sudo dhclient wlan0')

    def auto_connect(self, retry_time=30, on=True):
        def ac():
            r = retry_time
            while self._auto_connect:
                if not self.is_connected():
                    self.connect()
                    self.wait_for_change(timeout=r)
                    if self.is_connected():
                        r = max(2 * (time.time() - self.change_time()), retry_time)
                    else:
                        r *= 2
        self._ac_lock.acquire()
        if self._auto_connect != on:
            self._auto_connect = on
            if on:
                th = threading.Thread(target=ac)
                th.daemon = True
                th.start()
        self._ac_lock.release()

    def is_auto_connect(self):
        return self._auto_connect

    def is_reconnecting(self):
        return (not self.is_connected()) and self._auto_connect

    def is_connected(self):
        return self.con[-1][1]

    def change_time(self):
        return self.con[-1][0]


def main():
    log = WirelessLog()
    con = WirelessConnection()
    con.auto_connect()
    update_every = 5
    start_time = time.time()
    try:
        while True:
            t = (-int(time.time() - start_time) % update_every) or update_every
            if con.wait_for_change(t):
                log.write(('Disconnected' if con.is_connected() else "Connected")
                          (' for ' + str(int(con.con[-1][0] - con.con[-2][0])) + ' seconds\n'))
            log.display()
            sys.stdout.write("\033[K")
            dur = int(time.time() - con.change_time())
            if con.is_connected:
                print 'Connected for', dur, 'seconds'
            else:
                print 'Disconnected for', dur, 'seconds... Attempting to reconnect'
            sys.stdout.write("\033[F\r")
    finally:
        print
        log.write(('Connected' if con else "Disconnected") +
                  (' for ' + str(int(time.time() - con.change_time())) + ' seconds\n') +
                  ('-' * 80 + '\n')
                  )

if __name__ == "__main__":
    main()
