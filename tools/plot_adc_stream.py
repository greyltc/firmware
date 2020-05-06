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
import curses
import asciichartpy
from collections import deque

PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
ADC_BITS = 24
ADC_VREF = 2.048  # volts
ONE_LSB = ADC_VREF/(2**(ADC_BITS-1))  # volts per count
dt = 1036/2.048e6  # sample period
custom_timeout = 5 # 500 ms should pass until we give up on getting data we want
sam_len = 4  # number of bytes per sample (three for ADC counts, one for the counter value)

parser = argparse.ArgumentParser(description=f'Stream from ADC with sample interval {dt} [s]')
parser.add_argument('-s', '--server-hostname', type=str, default="WIZnet111785.lan",
                    help='hostname or IP address of server to connect to')
parser.add_argument('-n', '--num-samples', type=int, default=10001,
                    help='number of samples to stream')
parser.add_argument('-t', '--capture-duration', type=int, default=None,
                    help='number of seconds to capture for')
parser.add_argument('-l', '--live-plot', action="store_true",
                    help='make a live plot in the terminal, runs until the user presses "q"')

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


"""
gets one datapoint, happens to return my CPU temperature in this example
"""
def get_datapoint(s):
    some_data = bytes([])
    while len(some_data) < 4:
        try:
            some_data = some_data + s.recv(1)
        except socket.timeout:
            some_data = None
            break

    if some_data is not None:
        counts = int.from_bytes(some_data[1:4], byteorder='big', signed=True)
        counter = int(some_data[0])
    else:
        counts = None
        counter = None
    return( (counter, counts) )


class MyTelnet(Telnet):
    def read_response(self):
        return self.read_until(PROMPT,timeout=custom_timeout).rstrip(PROMPT).decode().strip()

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)


class Downsampler:
    """
    Feed this class high frequency data and every [factor]
    samples it will return an average of the last [factor] samples
    Can be used as an input filter to slow down datarate (and potentially increase precision)
    """
    def __init__(self, factor = 5):
        self.factor = factor
        self.cache = []
        self.next_sample = 0
    
    def feed(self, sample):
        self.next_sample += 1
        self.cache.append(sample)

        if self.next_sample == self.factor: # the cache is full, compute and return the average
            ret_val = sum(self.cache)/float(self.factor)
            self.next_sample = 0
            self.cache = []
        else:
            ret_val = None
        
        return ret_val

# handles terminal output
def cursed(stdscr, tn):
    # Clear screen
    stdscr.clear() # clear the screen
    stdscr.nodelay(True) # don't wait for input
    curses.curs_set(0) # hide the cursor
    last_counter = 0

    quit_key = 'q'

    # in characters 
    plot_width = 200
    plot_height = 40
    # TODO: read the terminal size with curses and use that

    average_window_length = 5 # length of running average window
    downsample_by = 10 # factor for downsampling

    display = deque([], plot_width) # what we'll be displaying

    cache = deque() # used in calculating the rolling mean
    cum_sum = 0 # used for calculating rolling mean
    ds = Downsampler(downsample_by)

    tn.send_cmd('stream0')  # send the capture command
    tn.read_until(b'...')  # get the stream start message
    tn.sock.settimeout(custom_timeout)

    while True:
        stdscr.erase()

        counter, counts = get_datapoint(tn.sock) # get a new datapoint
        if counter is None:
            break
        while (this_data := ds.feed(counts)) is None: # feed the downsampler with raw data until it gives us a data point
            counter, counts = get_datapoint(tn.sock) # get a new datapoint
            if counter is None:
                break
        if counter is None:
            break

        # do rolling average computation
        cache.append(this_data)
        cum_sum += this_data
        if len(cache) < average_window_length:
            pass
        else:
            cum_sum -= cache.popleft()
        this_avg = cum_sum/float(len(cache))

        # draw the plot
        to_display = this_avg * ONE_LSB * 1000000  # convert to microvolts
        #to_display = this_data
        stdscr.addstr(0, 0, f"Microvolts = {to_display}     ===== press {quit_key} to quit =====")
        display.append(to_display)
        stdscr.addstr(1, 0, asciichartpy.plot(display, {'height': plot_height}))
        stdscr.refresh()

        ch =  stdscr.getch()
        if ch == ord(quit_key): # ends the program
            break
        elif ch == ord('r'): # r key does nothing
            pass
    if counter is None:
        success = False
    else:
        success = True
    return (success)

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

    if args.live_plot:
        plot_width = 100
        plot_height = 30
        average_window_length = round(plot_width/5) # length of running average window
        downsample_by = 10 # factor for downsampling
        success = curses.wrapper(cursed, tn)


    else:
        tn.send_cmd(f'stream{args.num_samples}')  # send the capture command
        tn.read_until(b'...')  # get the stream start message
        print(f"\aStreaming for {args.capture_duration} seconds, that's {args.num_samples} samples at {1/dt} Hz...", end='', flush=True)

        start = time.time()
        raw_stream = sock_read_until(tn.sock, b'ADC streaming complete.').strip(b'ADC streaming complete.')
        end = time.time()
        elapsed = end-start

        leftover = tn.read_response()  # get the prompt

    # close/cleanup the connection
    tn.send_cmd('close')  # not strictly needed

if args.live_plot:
    if success is True:
        print("Capture aborted by user!")
    else:
        print("Timeout!")
else:
    print('done!')
    print(f'That took {elapsed} seconds, ({len(raw_stream)*8/elapsed/1000} kbps)')
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
