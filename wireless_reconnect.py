#! /usr/bin/python
import os
import sys
import time
import threading


class WirelessLog(object):
    def __init__(self, filename='connection.log'):
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
    def __init__(self, wait_time=1):
        def __check_connection():
            #return not os.system('ping -q -w9 -c1 8.8.8.8 >/dev/null 2>/dev/null')
            return not os.system('ping -q -w9 -c1 google.com >/dev/null 2>/dev/null')
        def __monitor_connection():
            while True:
                con = __check_connection()
                t = time.time()
                if con != self.con[-1][1]:
                    self.con.append((t, con))
                time.sleep(wait_time)
        con = __check_connection()
        self.con = [(time.time(), con)]
        self._auto_connect = False
        self._ac_lock = threading.Lock()
        th = threading.Thread(target=__monitor_connection)
        th.daemon = True
        th.start()

    def wait_for_change(self, timeout=float('inf')):
        "Returns True when connection state changes or False at timeout."
        t = time.time()
        while (self.change_time() <= t and time.time() < t + timeout):
            time.sleep(1)
        return self.change_time() > t

    def connect(self):
        if not self.is_connected():
            # nmcli dev wifi connect <SSID> iface <iface> > /dev/null 2> /dev/null
            # modprobe -r <driver> && modprobe <driver>
            os.system('ifconfig wlan0 down  >/dev/null 2>/dev/null && '
                      'ifconfig wlan0 up >/dev/null 2>/dev/null && '
                      'dhclient wlan0 >/dev/null 2>/dev/null ')
            os.system("systemctl stop wpa_supplicant")
            time.sleep(.1)

    def auto_connect(self, retry_time=90, on=True):
        def __ac():
            r = retry_time
            while self._auto_connect:
                if not self.is_connected():
                    self.connect()
                    self.wait_for_change(timeout=r)
                    if self.is_connected():
                        r = max(2 * (time.time() - self.change_time()), retry_time)
                    else:
                        r = min(2 * r, 60 * 4)
        self._ac_lock.acquire()
        if self._auto_connect != on:
            self._auto_connect = on
            if on:
                th = threading.Thread(target=__ac)
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
    if os.geteuid() != 0:
        exit("Script requires root.")
    log = WirelessLog()
    log.write('-' * 78 + '\n')
    con = WirelessConnection()
    con.auto_connect()
    update_every = 1
    start_time = time.time()
    source_path = os.path.dirname(os.path.abspath(__file__))
    try:
        while True:
            t = (-int(time.time() - start_time) % update_every) or update_every
            if con.wait_for_change(t):
                log.write(('Disconnected' if con.is_connected() else "Connected") +
                          (' for ' + str(int(con.con[-1][0] - con.con[-2][0])) + ' seconds\n'))
                if con.is_connected():
                    os.system('paplay ' + source_path + '/connect.ogg')
                else:
                    os.system('paplay ' + source_path + '/disconnect.ogg')
            log.display()
            sys.stdout.write("\033[K")
            dur = int(time.time() - con.change_time())
            if con.is_connected():
                print 'Connected for', dur, 'seconds'
            else:
                print 'Disconnected for', dur, 'seconds... Attempting to reconnect'
            sys.stdout.write("\033[F\r")
    finally:
        log.write(('Connected' if con.is_connected() else "Disconnected") +
                  (' for ' + str(int(time.time() - con.change_time())) + ' seconds\n'))
        print

if __name__ == "__main__":
    main()
