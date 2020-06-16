#!/usr/bin/env python3

from telnetlib import Telnet
import argparse
import socket
import time
import itertools
import sys
import random
import serial

PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
#default_host = "WIZnet111785.lan"
default_host = "10.46.0.233"

parser = argparse.ArgumentParser(description=f'Does stuff with the sage')
parser.add_argument('-s', '--server-hostname', type=str, default=default_host,
                    help='hostname or IP address of server to connect to')
parser.add_argument('-c', '--sourcemeter-comport', type=str, default="/dev/ttyUSB0",
                    help='Serial port for sourcemeter (57600 baud, term=<CR+LF>, flow control=on)')
parser.add_argument('-w', '--switch', action="store_true",
                    help='round-robin switch all switches')
parser.add_argument('-o', '--home', action="store_true",
                    help='homes the stage')
parser.add_argument('-b', '--bounce', action="store_true",
                    help='homes the stage then bounces it between 10 and 90 percent forever')
parser.add_argument('-a', '--axis', type=int, default=1,
                    help='axis to operate on')
parser.add_argument('-g', '--goto', type=int,
                    help='sends the stage somewhere')

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
        self.empty_response = False
        resp = self.read_until(PROMPT, timeout=timeout)
        ret = resp.rstrip(PROMPT).decode().strip()
        if len(resp) == 0:
            self.empty_response = True
        return ret

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)

# sends the stage somewhere.
# with retries, stall detection and timeout
# returns:
#  0 if the stage went there
# -1 if it did not go there and needs to be re-homed
# -2 if the timeout expired before it got there
# -3 is a programming error
def goto(tn, axis, position, timeout = 20):
    position = round(position) # must be a whole number of steps
    ret_val = -3
    retries = 5
    each_timeout = timeout/retries
    position_poll_freq = 0.5

    while retries > 0:
        tn.send_cmd(f'g{axis}{position:.0f}')
        resp = tn.read_response(timeout=1)
        if resp == '': # goto command accepted
            loc = None
            t0 = time.time()
            now = 0
            while (loc != position) and (now <= each_timeout): # poll for position
                tn.send_cmd(f'r{axis}') # ask for current position
                loc = int(tn.read_response(timeout=1))
                print(f'Location = {loc}') # for debugging
                time.sleep(position_poll_freq)
                now = time.time() - t0
            if now > each_timeout: # exited above loop because of microtimeout, retry
                ret_val = -2
                retries = retries - 1
            else: # we got there
                ret_val = 0
                break
        else: # goto command fail. this likely means the stage is unhomed either because it stalled or it just powered on
            ret_val = -1
            break
    return(ret_val)

# homes the stage
# returns:
# -1 if the homing timed out
# -2 if there was a programming error
# the length of the stage in steps for a successful home
def home(tn, axis, timeout = 80):
    ret_val = -2
    print('HOMING!')
    tn.send_cmd('h0')
    response = tn.read_response(timeout=timeout) # wait for homing to complete

    if response != '':
        raise(ValueError(f"Homing the stage failed: {response}"))
    else:
        if tn.empty_response == True: # we never got the prompt back
            ret_val = -2
        else:
            tn.send_cmd('l0')
            ret_val = int(tn.read_response())
    return(ret_val)


print(f"Connecting to {args.server_hostname}...", flush=True)
with MyTelnet(args.server_hostname) as tn:

    # get the welcome message
    welcome_message = tn.read_response(timeout=2)

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response(timeout=2)
    print(f"Got version request response: {version_message}")

    if args.bounce == True:
        home_result = home(tn, args.axis)
        if home_result <= 0:
            if home_result == -1:
                print ('Homing timed out.')
            else:
                raise(ValueError(f"Error homing stage: {home_result}"))
        else:
            print(f'The stage is {home_result} steps long.')

        length = home_result
        avg_stop_from_end = 0.1
        itc = itertools.cycle([length*avg_stop_from_end, length*(1-avg_stop_from_end)])
        while True:
            target = next(itc)
            # let's not bounce on the exact same place every time
            target = round(target + target*(random.random()*2 -1)*avg_stop_from_end*0.8)
            print(f"New target set: {target}")
            result = goto(tn, args.axis, target)
            if result !=0:
                if result == -1:
                    print ('We probably stalled and the stage. It should be re-homed\n"h" to rehome, anything else to quit')
                    user_in = input('>>> ')
                    if user_in == 'h':
                        home_result = home(tn, args.axis)
                        if home_result <= 0:
                            raise(ValueError("Homing failed: {home_result}"))
                        else:
                            print(f'The stage is {home_result} steps long.')
                            length = home_result
                            itc = itertools.cycle([round(length*0.1), round(length*0.9)])
                    else:
                        sys.exit(0)
                elif result == -2:
                    raise(ValueError("Timeout before reaching desired position"))
                else:
                    raise(ValueError("Error moving stage"))
            else:
                print(f"Bounced@{target}!")

    if args.switch == True:
        delay = 1
        rows = '1234'
        #rows = '1'
        cols = 'abcde'
        #cols = 'a'
        pixs = '123456'
        tn.send_cmd('c') # check what expanders we see
        expander_bitmask = tn.read_response(timeout=0.5)
        print(f"port expander bitmask = {expander_bitmask}")
        if expander_bitmask != '1023':
            raise(ValueError("Can't see all the port expanders"))
        tn.send_cmd('s') # deselect all
        tn.read_response(timeout=0.5)
        with serial.Serial(args.sourcemeter_comport, 57600, xonxoff=1) as ser:

            while True:
                for r in rows:
                    for c in cols:
                        for p in pixs:
                            cmd = f's{r}{c}{p}'
                            #print(cmd)
                            tn.send_cmd(cmd) # select pixel
                            tn.read_response(timeout=0.5)
                            # time.sleep(delay)
                            # sourcemeter needs to be all set up manually for this to work
                            # so maybe: measure ohms. output on.
                            # or: source current. measure voltage. output on.
                            ser.write('read?\r'.encode())
                            m = ser.readline() # expects '\n' (<LF> termination)
                            m = m.split(','.encode())
                            v = float(m[0])
                            i = float(m[1])
                            o = float(m[2])
                            print(f'{cmd}: {v} V, {i} A, {o} ohm')
                            tn.send_cmd('s') # deselect all
                            tn.read_response(timeout=0.5)
                        #time.sleep(delay)
                        print()


    if args.home == True:
        home_result = home(tn, args.axis)
        if home_result <= 0:
            if home_result == -1:
                print ('Homing timed out.')
            else:
                raise(ValueError(f"Error homing stage: {home_result}"))
        else:
            print(f'The stage is {home_result} steps long.')

    if args.goto:
        print(f'Going to {args.goto}')
        there = False
        while not there:
            result = goto(tn, args.axis, args.goto)
            if result !=0:
                if result == -1:
                    print ('We probably stalled and the stage. It should be re-homed\n"h" to rehome, anything else to quit')
                    user_in = input('>>> ')
                    if user_in == 'h':
                        home_result = home(tn, args.axis)
                        if home_result <= 0:
                            raise(ValueError("Homing failed: {home_result}"))
                        else:
                            print(f'The stage is {home_result} steps long.')
                    else:
                        sys.exit(0)
                elif result == -2:
                    raise(ValueError("Timeout before reaching desired position"))
                else:
                    raise(ValueError("Error moving stage"))
            else:
                there = True


print('done!')
