#!/usr/bin/env python3
"""
Python version of the RTES sequencer pattern.

  Sequencer  - 60 Hz tick, absolute-deadline timing, releases services
  Service 1  - 30 Hz  camera + sensors  (every 2 ticks)
  Service 2  - 10 Hz  motor command over serial to Arduino (every 6 ticks)
  Service 3  -  2 Hz  stats / logging (every 30 ticks)

Run:  sudo python3 sequencer.py --port /dev/ttyACM0 --seconds 30
      (sudo or CAP_SYS_NICE is needed for SCHED_FIFO; without it the script
       still runs and just warns.)
No --port => serial is simulated so you can check timing by itself.
"""
import argparse
import os
import threading
import time

try:
    import serial          # pip install pyserial
except ImportError:
    serial = None

TICK_HZ = 60
TICK_NS = 1_000_000_000 // TICK_HZ


# ---------------------------------------------------------------- RT helpers
def set_rt(priority, core):
    """Applies to the *calling thread* on Linux (pid 0 == current tid)."""
    try:
        os.sched_setaffinity(0, {core})
    except OSError as e:
        print(f"affinity failed: {e}")
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
    except PermissionError:
        print("SCHED_FIFO denied (run with sudo / CAP_SYS_NICE); continuing with normal scheduling")


RT_MAX = os.sched_get_priority_max(os.SCHED_FIFO)


# ---------------------------------------------------------------- shared data
class Latest:
    """Latest-value-wins slot. Services share data through this, never queues."""
    def __init__(self):
        self._lock = threading.Lock()
        self._val = None
        self._stamp = 0

    def put(self, v):
        with self._lock:
            self._val = v
            self._stamp = time.monotonic_ns()

    def get(self):
        with self._lock:
            return self._val, self._stamp


sensor_state = Latest()


# ---------------------------------------------------------------- service base
class Service(threading.Thread):
    def __init__(self, name, every_ticks, prio, core, work):
        super().__init__(name=name, daemon=True)
        self.every = every_ticks
        self.prio, self.core, self.work = prio, core, work
        self.evt = threading.Event()
        self.stop_flag = False
        self.count = 0
        self.overruns = 0       # sequencer tried to release while still running
        self.wcet_ns = 0
        self.sum_ns = 0

    def release(self):
        if self.evt.is_set():   # previous release not yet consumed => deadline miss
            self.overruns += 1
        self.evt.set()

    def run(self):
        set_rt(self.prio, self.core)
        while True:
            self.evt.wait()
            self.evt.clear()
            if self.stop_flag:
                return
            t0 = time.perf_counter_ns()
            self.work()
            dt = time.perf_counter_ns() - t0
            self.count += 1
            self.sum_ns += dt
            self.wcet_ns = max(self.wcet_ns, dt)


# ---------------------------------------------------------------- the work
class Hardware:
    def __init__(self, port, baud=115200):
        self.ser = None
        self.cap = None
        self.serial_drops = 0
        self.rx_buf = b""
        self.last_rx = ""
        self.write_errors = 0

        try:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # don't queue old frames
        except Exception as e:
            print(f"camera unavailable ({e}); simulating")

        if port and serial:
            # timeout=0 => non-blocking reads; write_timeout stops a stuck write
            # from freezing the service forever
            self.ser = serial.Serial(port, baud, timeout=0, write_timeout=0.02)
            time.sleep(2.0)   # most Arduinos reset when the port opens (DTR)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    # ---- Service 1: 30 Hz camera + sensors
    def service_sense(self):
        frame_ok, err = False, 0.0
        if self.cap is not None:
            frame_ok, frame = self.cap.read()
            if frame_ok:
                gray = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2GRAY)
                # placeholder vision: horizontal offset of brightest column
                err = (float(gray.mean(axis=0).argmax()) - gray.shape[1] / 2) / (gray.shape[1] / 2)
        sensor_state.put({"frame_ok": frame_ok, "err": err})

    # ---- Service 2: 10 Hz motor command
    def service_motor(self):
        state, stamp = sensor_state.get()
        if state is None:
            return
        # stale data check: don't act on a reading older than 100 ms
        if (time.monotonic_ns() - stamp) > 100_000_000:
            left = right = 0
        else:
            base = 120
            k = 60
            left = int(base + k * state["err"])
            right = int(base - k * state["err"])

        if self.ser is None:
            return
        try:
            # 1) drain anything the Arduino sent so ITS tx doesn't block and OUR rx doesn't fill
            n = self.ser.in_waiting
            if n:
                self.rx_buf += self.ser.read(n)
                *lines, self.rx_buf = self.rx_buf.split(b"\n")
                if lines:
                    self.last_rx = lines[-1].decode(errors="replace").strip()
                self.rx_buf = self.rx_buf[-256:]       # never grow unbounded

            # 2) if previous commands haven't gone out, drop them: newest wins
            if self.ser.out_waiting > 32:
                self.ser.reset_output_buffer()
                self.serial_drops += 1

            # 3) short, fixed, newline-framed command
            self.ser.write(f"M,{left},{right}\n".encode())
        except Exception:
            self.write_errors += 1

    # ---- Service 3: 2 Hz stats (set later, needs references)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--seconds", type=float, default=30)
    args = ap.parse_args()

    hw = Hardware(args.port)

    s1 = Service("sense", 2, RT_MAX - 1, 2, hw.service_sense)
    s2 = Service("motor", 6, RT_MAX - 2, 3, hw.service_motor)

    seq_stats = {"ticks": 0, "missed": 0, "max_jitter_ns": 0, "sum_jitter_ns": 0}

    def service_stats():
        print(f"[stats] ticks={seq_stats['ticks']} missed={seq_stats['missed']} "
              f"jitter max={seq_stats['max_jitter_ns']/1e3:.0f}us "
              f"avg={seq_stats['sum_jitter_ns']/max(1, seq_stats['ticks'])/1e3:.0f}us | "
              f"S1 wcet={s1.wcet_ns/1e6:.2f}ms over={s1.overruns} | "
              f"S2 wcet={s2.wcet_ns/1e6:.2f}ms over={s2.overruns} "
              f"serial_drops={hw.serial_drops} werr={hw.write_errors} rx='{hw.last_rx}'")

    s3 = Service("stats", 30, RT_MAX - 3, 3, service_stats)
    services = [s1, s2, s3]
    for s in services:
        s.start()

    # ---------------- Sequencer (runs in main thread, highest priority)
    set_rt(RT_MAX, 1)
    start = time.monotonic_ns()
    next_deadline = start + TICK_NS
    end = start + int(args.seconds * 1e9)
    tick = 0

    while time.monotonic_ns() < end:
        # sleep until the ABSOLUTE deadline (no drift accumulation)
        remaining = next_deadline - time.monotonic_ns()
        if remaining > 0:
            time.sleep(remaining / 1e9)

        woke = time.monotonic_ns()
        jitter = woke - next_deadline
        seq_stats["max_jitter_ns"] = max(seq_stats["max_jitter_ns"], jitter)
        seq_stats["sum_jitter_ns"] += max(jitter, 0)

        tick += 1
        seq_stats["ticks"] = tick
        for s in services:
            if tick % s.every == 0:
                s.release()

        next_deadline += TICK_NS
        if woke > next_deadline:               # we fell more than a tick behind
            skipped = (woke - next_deadline) // TICK_NS + 1
            seq_stats["missed"] += skipped
            next_deadline += skipped * TICK_NS  # resync instead of bursting to catch up

    for s in services:
        s.stop_flag = True
        s.evt.set()
    for s in services:
        s.join(timeout=1)
    print("DONE")
    service_stats()


if __name__ == "__main__":
    main()
