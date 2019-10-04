#!/usr/bin/env python

# cool gnome tip for making the bell visual:
# gsettings set org.gnome.desktop.wm.preferences visual-bell true
# gsettings set org.gnome.desktop.wm.preferences visual-bell-type fullscreen-flash

from telnetlib import Telnet
import numpy
import matplotlib
matplotlib.use('GTK3Agg')  # or 'GTK3Cairo'
import matplotlib.pyplot as plt
import socket
import argparse
import time
import sys

PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
ADC_BITS = 24
ADC_VREF = 2.048  # volts
ONE_LSB = ADC_VREF/(2**(ADC_BITS-1))  # volts per count
dt = 1036/2.048e6  # sample period
custom_timeout = 5 # 500 ms should pass until we give up on getting data we want

parser = argparse.ArgumentParser(description=f'Stream from ADC with sample interval {dt} [s]')
parser.add_argument('-s', '--server-hostname', type=str, default="WIZnet111785.desk_lan",
                    help='hostname or IP address of server to connect to')
parser.add_argument('-n', '--num-samples', type=int, default=10001,
                    help='number of samples to stream')
parser.add_argument('-t', '--capture-duration', type=int, default=None,
                    help='number of seconds to capture for')

args = parser.parse_args()

# the capture duration argument overrides the number of samples argument
if args.capture_duration is not None:
    args.num_samples = round(args.capture_duration/dt)+1
else:
    args.capture_duration = (args.num_samples-1)*dt


def sock_read_until(s, match):
    old_tout = s.gettimeout()
    s.settimeout(custom_timeout)
    matched = False
    buffer = bytes([])
    while (matched is False):
        try:
            buffer = buffer + s.recv(1)
        except socket.timeout:
            print("WARNING: Data reception timeout")
            break
        if buffer.endswith(match):
            matched = True
    s.settimeout(old_tout)
    return buffer


class MyTelnet(Telnet):
    def read_response(self):
        return self.read_until(PROMPT,timeout=custom_timeout).rstrip(PROMPT).decode().strip()

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)


print(f"Connecting to {args.server_hostname}...", flush=True)
with MyTelnet(args.server_hostname) as tn:
    # get the welcome message
    welcome_message = tn.read_response()
    print(f"Connected with welcome message: {welcome_message}")

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response()
    print(f"Got version request response: {version_message}")

    tn.send_cmd(f'stream{args.num_samples}')  # send the capture command
    tn.read_until(b'interval...')  # get the stream start message
    print(f"\aStreaming for {args.capture_duration} seconds, that's {args.num_samples} samples at {1/dt} Hz...", end='', flush=True)

    start = time.time()
    raw_stream = sock_read_until(tn.sock, b'ADC streaming complete.').strip(b'ADC streaming complete.')
    end = time.time()
    elapsed = end-start

    print('done!')
    print(f'That took {elapsed} seconds, ({len(raw_stream)*8/elapsed/1000} kbps)')
    leftover = tn.read_response()  # get the prompt

    # close/cleanup the connection
    tn.send_cmd('close')  # not strictly needed

sam_len = 4  # number of bytes per sample
raw_samples = [raw_stream[i*sam_len:i*sam_len+sam_len] for i in range(len(raw_stream)//sam_len)]
counts = numpy.array([int.from_bytes(raw_samples[i][1:4], byteorder='big', signed=True) for i in range(len(raw_samples))])
counter = numpy.array([int(raw_samples[i][0]) for i in range(len(raw_samples))], dtype=numpy.uint8)
#crcs = numpy.array([int.from_bytes(raw_samples[i][4:6],byteorder=sys.byteorder,signed=False) for i in range(len(raw_samples))])
diff = numpy.diff(counter).astype(int)  # differences between the counter values
diff = numpy.concatenate(([1], diff))  # left append a 1 for the first value
misses = sum(diff-1)  # count up the missed samples
got_n_samples = len(raw_samples)
n_missed = args.num_samples - got_n_samples
if misses == 0 and got_n_samples == args.num_samples:
    print(f"Got {got_n_samples} samples with no misses!")
else:
    print(f"WARNING: Got {got_n_samples} with {misses} sample(s) skipped! (that's {n_missed} less than expected)")

v = counts * ONE_LSB
t = (numpy.cumsum(diff)-1) * dt
if got_n_samples > 0:
    print(f"Plotting from time t=0 to t={t.max()} seconds")
    plt.plot(t, v, marker='.')
    plt.ylabel('Voltage [V]')
    plt.xlabel('Time [s]')
    plt.show()
