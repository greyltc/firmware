#!/usr/bin/env python3

from telnetlib import Telnet
import argparse
import socket
import time
import itertools

PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects


parser = argparse.ArgumentParser(description=f'Does stuff with the sage')
parser.add_argument('-s', '--server-hostname', type=str, default="WIZnet111785.lan",
                    help='hostname or IP address of server to connect to')
parser.add_argument('-o', '--home', action="store_true",
                    help='homes the stage')
parser.add_argument('-b', '--bounce', action="store_true",
                    help='homes the stage then bounces it between 10 and 90 percent forever')


args = parser.parse_args()

def sock_read_until(s, match, timeout=None):
    old_tout = s.gettimeout()
    s.settimeout(timeout)
    matched = False
    buffer = bytes([])
    while (matched is False):
        try:
            buffer = buffer + s.recv(1)
        except socket.timeout:
            break
        if buffer.endswith(match):
            matched = True
    s.settimeout(old_tout)
    return buffer

class MyTelnet(Telnet):
    def read_response(self, timeout=None):
        resp = self.read_until(PROMPT,timeout=timeout)
        ret = resp.rstrip(PROMPT).decode().strip()
        if len(resp) == 0:
            raise(ValueError("Got no response"))
        return ret

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)


print(f"Connecting to {args.server_hostname}...", flush=True)
with MyTelnet(args.server_hostname) as tn:
    # get the welcome message
    welcome_message = tn.read_response(timeout=2)

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response(timeout=2)
    print(f"Got version request response: {version_message}")

    if args.home == True:
        print('HOMING!')
        tn.send_cmd('h0')
        tn.read_response()
    
    if args.bounce == True:
        print('HOMING!')
        tn.send_cmd('h0')
        tn.read_response() # wait for homing to complete
        print('BOUNCING!')
        tn.send_cmd('l0')
        length = int(tn.read_response())
        itc = itertools.cycle([round(length*0.1), round(length*0.9)])
        while True:
            target = next(itc)
            tn.send_cmd(f'g0{target:.0f}')
            tn.read_response()
            while True:
                tn.send_cmd(f'r0')
                loc = int(tn.read_response())
                if loc == target:
                    break
                else:
                    print(f'Location = {loc}')
                    time.sleep(0.2)


print('done!')