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
from signal import signal, SIGWINCH
import asciichartpy
from collections import deque

PROMPT = b'>>> '  # the firmware's prompt
EOL = b'\r\n'  # client-->server transmission line ending the firmware expects
ADC_BITS = 24
ADC_VREF = 2.048  # volts
ONE_LSB = ADC_VREF/(2**(ADC_BITS-1))  # volts per count
dt = 1036/2.048e6  # sample period
sam_len = 4  # number of bytes per sample (three for ADC counts, one for the counter value)

parser = argparse.ArgumentParser(description=f'Stream from ADC with sample interval {dt} [s]')
parser.add_argument('-s', '--server-hostname', type=str, default="WIZnet111785.lan",
                    help='hostname or IP address of server to connect to')
parser.add_argument('-n', '--num-samples', type=int, default=10001,
                    help='number of samples to stream')
parser.add_argument('-t', '--capture-duration', type=int, default=None,
                    help='number of seconds to capture for')
parser.add_argument('-p', '--static-plot', action="store_true",
                    help='records data silently then plots it')

args = parser.parse_args()

# the capture duration argument overrides the number of samples argument
if args.capture_duration is not None:
    args.num_samples = round(args.capture_duration/dt)+1
else:
    args.capture_duration = (args.num_samples-1)*dt


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


"""
returns the ADC counts (24 bit sigend int) and the counter (1 byte unsigend int)
"""
def get_datapoint(s):
    some_data = bytes([])
    while len(some_data) < 4:
        try:
            some_data = some_data + s.recv(1)
        except socket.timeout:
            raise(ValueError(f"datapoint timeout: {s.gettimeout()}"))
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
    def read_response(self, timeout=None):
        resp = self.read_until(PROMPT,timeout=timeout).rstrip(PROMPT).decode().strip()
        if len(resp) == 0:
            raise(ValueError("Got no response"))
        return resp

    def send_cmd(self, cmd):
        return self.write(cmd.encode()+EOL)


class Downsampler:
    """
    Feed this class high frequency data and every [factor]
    samples it gets, it will return an average of the last [factor] samples
    Can be used as an input filter to slow down data rate (and potentially increase precision)
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

class ProcessPipeline:
    """feed this class raw data points and it will return a deque for plotting
    does downsampling and rolling window averaging"""

    def __init__(self, output_len, ds_factor = 10, rolling_avg_window = 5):
        self.ds_factor = ds_factor
        self.rolling_avg_window = rolling_avg_window
        self.ds = Downsampler(self.ds_factor) # init the downsampler
        self.output = deque([], output_len) # holds the output data
        self.cache = deque() # used in calculating the rolling mean
        self.cum_sum = 0 # used for calculating rolling mean

    # either returns True if self.outout has been updated, otherwise False
    def feed(self, raw):
        updated = False
        # do rolling average computation
        if (d := self.ds.feed(raw)) is not None:
            self.cache.append(d)
            self.cum_sum += d
            if len(self.cache) < self.rolling_avg_window:
                pass
            else:
                self.cum_sum -= self.cache.popleft()
            self.output.append(self.cum_sum/float(len(self.cache)))
            updated = True
        return updated

# for handling the curses output
class Cursed:
    def __init__(self, tn):
        self.tn = tn
        self.quit_key = 'q'
        self.__dict__["cols"] = None # init as None so that the custom __setattr__ below works
        self.x_padding = 14  # this depends on the width of the y axis labels...
        self.y_padding = 5
        self.ds_factor = 20
        self.rolling_avg_window = 5
        #curses.curs_set(0) # hide the cursor
        self.display_scale_factor = ONE_LSB * 1000000  # convert to microvolts

    def __setattr__(self, name, value):
        if name=="cols": # changing cols means changing the data processing pipeline
            if self.cols != value:
                super().__setattr__(name, value)
                self.pp = ProcessPipeline(value-self.x_padding, ds_factor=self.ds_factor, rolling_avg_window=self.rolling_avg_window)
        else:
            super().__setattr__(name, value)

    def run(self):
        self.tn.send_cmd('stream0')  # send the stream forever command
        self.tn.read_until(b'...', timeout=2)  # get the stream start message
        #self.tn.sock.settimeout(custom_timeout)
        return curses.wrapper(self.do_stuff_with_curses)

    def resize_handler(self, signum, frame):
        curses.endwin()
        self.stdscr.refresh()
        self.rows, self.cols = self.stdscr.getmaxyx() # triggers a new self.pp of cols changes
        #self.stdscr.refresh()
    
    # returns number of missed counts
    def count_missed(self, new_counter):
        #self.cnts.append(new_counter)
        missed = 0
        if self.last_counter is not None:
            missed = ((new_counter - self.last_counter) & 0xff) - 1
        self.total_missed += missed
        self.total += missed + 1
        self.last_counter = new_counter
        return missed

    def do_stuff_with_curses(self, stdscr):
        self.stdscr = stdscr
        signal(SIGWINCH, self.resize_handler) # connect up resize handler function
        stdscr.clear() # clear the screen
        stdscr.nodelay(True) # don't wait for input
        curses.curs_set(0) # hide the cursor
        self.rows, self.cols = stdscr.getmaxyx() # triggers a new self.pp
        self.last_counter = None # last counter value from the ADC
        self.total_missed = 0 # total samples missed conversions so far
        self.total = 0# total number of samples seen by the ADC
        #self.cnts = []
    
        while True:
            stdscr.erase() # or else the plot leaves garbage behind

            counter, counts = get_datapoint(self.tn.sock) # get a new datapoint
            if counter is None:
                break # break out if there was a timeout
            self.count_missed(counter)
            while self.pp.feed(counts) == False:
                counter, counts = get_datapoint(self.tn.sock) # get a new datapoint
                if counter is None:
                    break # break out if there was a timeout
                self.count_missed(counter)
            if counter is None:
                break# break out if there was a timeout
            
            to_display = [self.display_scale_factor*v for v in self.pp.output]
            try:
                stdscr.addstr(0, 0, f"=== press {self.quit_key} to quit ===")
            except:
                print('Error: The terminal window must be wide enough to fit the quit key message.')
            try: # these might fail during window resize
                stdscr.addstr(1, 0, f"Microvolts = {to_display[0]}")
                stdscr.addstr(2, 0, f"Conversions [total, missed, percentage] = [{self.total}, {self.total_missed}, {self.total_missed/self.total*100}%]")
                stdscr.addstr(3, 0, asciichartpy.plot(to_display, {'height': self.rows-self.y_padding}))
            except:
                    pass
            stdscr.refresh()

            ch =  stdscr.getch()
            if ch == ord(self.quit_key): # ends the program
                break
            elif ch == ord('r'): # r key does nothing
                pass
        if counter is None: # exited due to timeout
            success = False
        else: # exited due to user pressed quit key
            success = True 
        return (success)


print(f"Connecting to {args.server_hostname}...", flush=True)
with MyTelnet(args.server_hostname) as tn:
    # get the welcome message
    welcome_message = tn.read_response(timeout=2)

    # test a command (ask for virmware revision)
    # tn.write(b'v'+EOL)
    tn.send_cmd('v')
    version_message = tn.read_response(timeout=2)
    print(f"Got version request response: {version_message}")

    if not args.static_plot: # live plot
        c = Cursed(tn)
        success = c.run()
    else: # static plot
        tn.send_cmd(f'stream{args.num_samples}')  # send the capture command
        tn.read_until(b'...', timeout=2)  # get the stream start message
        print(f"\aStreaming for {args.capture_duration} seconds, that's {args.num_samples} samples at {1/dt} Hz...", end='', flush=True)

        start = time.time()
        raw_stream = sock_read_until(tn.sock, b'ADC streaming complete.',timeout=2).strip(b'ADC streaming complete.')
        end = time.time()
        elapsed = end-start
        try: # this wil fail if there was a timeout above
            leftover = tn.read_response(timeout=2)  # get the prompt
        except:
            pass

    # close/cleanup the connection
    tn.send_cmd('close')  # not strictly needed

if not args.static_plot: # live plot
    if success is True:
        print("Capture aborted by user!")
    else:
        print("Timeout!")
else: # static plot
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
