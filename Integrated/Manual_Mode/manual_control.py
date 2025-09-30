
import sys, threading, time, queue, termios, tty, select

class ManualController:
    """
    Polls keyboard for:
      - w/a/s/d → fine motion deltas
      - SHIFT + w/a/s/d → coarse motion deltas
      - 'm' → toggle MANUAL <-> AUTO (via callback)
      - 'q' → optional quick park to home (callback)
    Emits motion commands as (d_az, d_el) in PHYSICAL degrees.
    """
    def __init__(self, on_toggle_mode, on_quick_park=None,
                step_deg=2.0, step_deg_fast=8.0, poll_hz=60.0):
                self._on_toggle_mode = on_toggle_mode
                self._on_quick_park = on_quick_park
                self._step = float(step_deg)
                self._step_fast = float(step_deg_fast)
                self._poll = 1.0 / float(poll_hz)
                self._q = queue.Queue(maxsize=64)
                self._stop = threading.Event()
                self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def read_command(self, timeout=0.0):
        try:
            return self._q.get(timeout=timeout)  # (daz, del)
        except queue.Empty:
            return (0.0, 0.0)

    # -------- internals --------
    def _getch_nonblock(self):
        if not sys.stdin.isatty():
            time.sleep(self._poll)
            return None, False

        dr,_,_ = select.select([sys.stdin], [], [], self._poll)
        if not dr:
            return None, False
        ch = sys.stdin.read(1)
        # quick detect “shifted” by peeking for uppercase
        is_fast = ch.isupper()
        return ch.lower(), is_fast

    def _run(self):
        # put stdin into raw mode
        if sys.stdin.isatty():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        else:
            old = None

        try:
            while not self._stop.is_set():
                ch, fast = self._getch_nonblock()
                if ch is None:
                    continue
                step = self._step_fast if fast else self._step

                if ch == 'w':    # up (increase elevation)
                    self._emit(0.0, +step)
                elif ch == 's':  # down (decrease elevation)
                    self._emit(0.0, -step)
                elif ch == 'a':  # left (decrease azimuth)
                    self._emit(-step, 0.0)
                elif ch == 'd':  # right (increase azimuth)
                    self._emit(+step, 0.0)
                elif ch == 'm':  # toggle mode
                    if self._on_toggle_mode: self._on_toggle_mode()
                elif ch == 'q':  # optional quick-park
                    if self._on_quick_park: self._on_quick_park()
                # ignore everything else
        finally:
            if old is not None:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)

    def _emit(self, daz, delv):
        try:
            self._q.put_nowait((float(daz), float(delv)))
        except queue.Full:
            pass
