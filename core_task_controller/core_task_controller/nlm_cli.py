#!/usr/bin/env python3
"""Full-screen terminal client for nlm_interface.

Type a command, watch what the LLM made of it and what the robot does next,
without juggling three terminals of `ros2 topic pub` and `topic echo`.

Thin client on purpose: it publishes to /natural_language_mission and renders
/nlm_feedback and /operation_feedback. It never calls Claude, so it needs no
API key - nlm_interface owns that.

    ros2 run core_task_controller nlm_cli
"""
import curses
import os
import textwrap
import threading
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from geometry_msgs.msg import Twist

HELP = ('enter=send | abort=stop | save <name>/done=finish | '
        'reset=respawn | kill=teardown | /quit=exit')
# Tag -> (label, colour pair id). Colour is set up in _init_colours.
SENT, OK, BAD, PHASE, INFO = 'sent', 'ok', 'bad', 'phase', 'info'

# Abort is a safety command, so it is matched locally and calls the executor
# service directly - never routed through the LLM, which could stall or
# misread it. Kept deliberately small and explicit.
_ABORT_PHRASES = (
    'abort', 'abort mission', 'stop', 'halt', 'cancel',
    'return to start', 'return to dock', 'go home', 'come home',
)

# "I'm finished" for the operator-driven phases (mapping, goal collection).
# Calls /operator_done directly - the LLM cannot express this as a mission, so
# routing it through the model just produces "no json block" rejections.
# Single-word finish verbs; may take a map name in the mapping phase.
_DONE_VERBS = (
    'done', 'save', 'saved', 'finish', 'finished', 'complete', 'completed',
)
# Multi-word finish phrases (collection; never take a name).
_DONE_PHRASES = ('operator done', 'done collecting', 'stop collecting')


def is_abort(text):
    """Return True if `text` is a stop/come-home command, not a mission."""
    return text.strip().lower().lstrip('/') in _ABORT_PHRASES


def parse_finish(text):
    """Parse a finish command. Returns the map-name argument ('' if none) when
    `text` is one, else None. 'save floor1' -> 'floor1'; 'done' -> ''."""
    t = text.strip().lstrip('/')
    if t.lower() in _DONE_PHRASES:
        return ''
    parts = t.split(None, 1)
    if parts and parts[0].lower() in _DONE_VERBS:
        return parts[1].strip() if len(parts) > 1 else ''
    return None


class _Bus(Node):
    """ROS side of the CLI: publishes commands, collects feedback."""

    def __init__(self, sink):
        """Wire up the command publisher and the two feedback subscriptions."""
        super().__init__('nlm_cli')
        self._sink = sink
        self.phase = '-'
        self.pub = self.create_publisher(String, 'natural_language_mission', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(String, 'nlm_feedback', self._on_feedback, 10)
        # /operation_feedback is published TRANSIENT_LOCAL (latched). Match it,
        # otherwise the status bar shows '-' until the next phase change rather
        # than the phase the robot is already in.
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, 'operation_feedback',
                                 self._on_phase, latched)
        self.abort_cli = self.create_client(Trigger, 'abort_mission')
        self.done_cli = self.create_client(Trigger, 'operator_done')
        self.save_pub = self.create_publisher(String, 'save_map', 10)
        # Gazebo's world reset: teleports every model back to its spawn pose.
        # Provided by gazebo_ros_init at the root namespace, not under /gazebo.
        self.reset_cli = self.create_client(Empty, '/reset_world')
        # Executor's FSM reset: clears a held FAULT back to IDLE.
        self.reset_fsm_cli = self.create_client(Trigger, 'reset')

    def send(self, text):
        """Publish a natural language command."""
        self.pub.publish(String(data=text))

    def save_map(self, name):
        """Publish the map name; the executor saves the SLAM map under it."""
        self.save_pub.publish(String(data=name))
        self._sink(SENT, 'SAVE MAP "%s"' % name)

    def abort(self):
        """Call /abort_mission and report the executor's reply on the transcript."""
        if not self.abort_cli.service_is_ready():
            self._sink(BAD, 'abort: executor not running (/abort_mission absent)')
            return
        self._sink(SENT, 'ABORT')

        def done(future):
            try:
                resp = future.result()
            except Exception as exc:                       # noqa: BLE001
                self._sink(BAD, 'abort failed: %s' % exc)
                return
            self._sink(OK if resp.success else BAD, 'abort: %s' % resp.message)

        self.abort_cli.call_async(Trigger.Request()).add_done_callback(done)

    def reset(self):
        """`reset`: models back to spawn (/reset_world) and, if the executor is
        faulted, its FSM back to IDLE. Fires both; each is independent."""
        self._sink(SENT, 'RESET')
        if self.reset_cli.service_is_ready():
            def world_cb(future):
                try:
                    future.result()
                except Exception as exc:                   # noqa: BLE001
                    self._sink(BAD, 'reset world failed: %s' % exc)
                    return
                self._sink(OK, 'reset: models back at spawn')
            self.reset_cli.call_async(Empty.Request()).add_done_callback(world_cb)
        else:
            self._sink(BAD, 'reset: Gazebo not running (/reset_world absent)')

        if self.reset_fsm_cli.service_is_ready():
            def fsm_cb(future):
                try:
                    resp = future.result()
                except Exception as exc:                   # noqa: BLE001
                    self._sink(BAD, 'reset fsm failed: %s' % exc)
                    return
                # Not-faulted comes back success=False; that's informational,
                # not an error, so tag it INFO rather than BAD.
                self._sink(OK if resp.success else INFO, 'reset: %s' % resp.message)
            self.reset_fsm_cli.call_async(
                Trigger.Request()).add_done_callback(fsm_cb)

    def done(self):
        """Call /operator_done (finish mapping or goal collection) and report."""
        if not self.done_cli.service_is_ready():
            self._sink(BAD, 'done: executor not running (/operator_done absent)')
            return
        self._sink(SENT, 'DONE')

        def cb(future):
            try:
                resp = future.result()
            except Exception as exc:                       # noqa: BLE001
                self._sink(BAD, 'done failed: %s' % exc)
                return
            self._sink(OK if resp.success else BAD, 'done: %s' % resp.message)

        self.done_cli.call_async(Trigger.Request()).add_done_callback(cb)

    def listener_count(self):
        """Return how many nodes are listening for our commands."""
        return self.pub.get_subscription_count()

    def api_ready(self):
        """Check if any LLM provider is configured."""
        return bool(os.getenv("OPENAI_API_KEY") or
                    os.getenv("ANTHROPIC_API_KEY") or
                    os.getenv("AWS_BEARER_TOKEN_BEDROCK") or
                    os.getenv("AWS_ACCESS_KEY_ID"))

    def teleop(self, linear, angular):
        """Publish a cmd_vel for direct robot control (teleop)."""
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def _on_feedback(self, msg):
        tag = OK if msg.data.startswith('Mission accepted') else BAD
        self._sink(tag, msg.data)

    def _on_phase(self, msg):
        previous, self.phase = self.phase, msg.data
        self._sink(PHASE, msg.data)
        # Entering FAULT: the executor holds and waits. Only a fresh mission
        # re-initialises it - abort/return-to-start are no-ops here - so say so
        # instead of leaving the operator guessing at a stuck 'fault' phase.
        if msg.data.startswith('fault') and not previous.startswith('fault'):
            self._sink(BAD, 'faulted - type "reset" to clear back to idle, or '
                            'send a new mission (e.g. "patrol the perimeter '
                            'twice"); abort / return to start do nothing here.')


class _Screen:
    """curses rendering: full-screen UI with header, sections, status, and feedback."""

    def __init__(self, stdscr, bus):
        """Set up colours and the shared transcript buffer."""
        self.stdscr = stdscr
        self.bus = bus
        self.lines = deque(maxlen=500)
        self.lock = threading.Lock()
        self.buf = ''
        self.squad_mode = False
        self.last_teleop_time = 0
        curses.curs_set(1)
        stdscr.nodelay(True)
        self._init_colours()

    def _init_colours(self):
        self.colour = {}
        if not curses.has_colors():
            return
        curses.start_color()
        curses.use_default_colors()
        for i, (tag, fg) in enumerate((
            (SENT, curses.COLOR_WHITE), (OK, curses.COLOR_GREEN),
            (BAD, curses.COLOR_RED), (PHASE, curses.COLOR_CYAN),
            (INFO, curses.COLOR_YELLOW),
        ), start=1):
            curses.init_pair(i, fg, -1)
            self.colour[tag] = curses.color_pair(i)

    def add(self, tag, text):
        """Append a tagged line to the transcript. Safe from any thread."""
        with self.lock:
            self.lines.append((tag, text))

    def _attr(self, tag):
        a = self.colour.get(tag, 0)
        return a | curses.A_BOLD if tag in (SENT, OK, BAD) else a

    def _wrapped(self, width):
        out = []
        with self.lock:
            rows = list(self.lines)
        for tag, text in rows:
            prefix = {SENT: '> ', OK: '  ok      ', BAD: '  rejected ',
                      PHASE: '  phase   ', INFO: '  '}[tag]
            body = textwrap.wrap(text, max(8, width - len(prefix) - 1)) or ['']
            out.append((tag, prefix + body[0]))
            out.extend((tag, ' ' * len(prefix) + b) for b in body[1:])
        return out

    def draw(self):
        """Repaint the whole screen with formatted layout and feedback panel."""
        # Check teleop timeout: stop robot if no movement for 200ms
        if self.last_teleop_time and time.time() - self.last_teleop_time > 0.2:
            self.bus.teleop(0.0, 0.0)
            self.last_teleop_time = 0

        s = self.stdscr
        h, w = s.getmaxyx()
        if h < 20 or w < 65:
            s.erase()
            try:
                s.addnstr(0, 0, 'Terminal too small (need 65x20+)', w - 1)
            except curses.error:
                pass
            s.refresh()
            return
        s.erase()

        row = 0
        # Top border
        self._draw_line(row, w, '╔', '═', '╗')
        row += 1

        # Title
        title = 'OMOKAI MISSION PROMPT'
        pad = (w - 2 - len(title)) // 2
        line = '║ ' + ' ' * pad + title + ' ' * (w - 3 - pad - len(title)) + '║'
        self._draw_text(row, line, curses.A_BOLD)
        row += 1

        # Section divider
        self._draw_line(row, w, '╠', '═', '╣')
        row += 1

        # MISSIONS section
        self._draw_text(row, '║ MISSIONS (plain English):', 0)
        row += 1
        missions = [
            'patrol the perimeter twice in warehouse',
            'start building a map called mapname',
            'collect waypoints',
            'both robots patrol the perimeter twice in warehouse'
        ]
        for mission in missions:
            line = '║   • ' + mission.ljust(w - 8) + '║'
            self._draw_text(row, line, 0)
            row += 1

        # Blank line
        self._draw_text(row, '║' + ' ' * (w - 2) + '║', 0)
        row += 1

        # CONTROLS section
        controls = 'abort | return to start | save | done | reset | kill'
        self._draw_text(row, '║ CONTROLS: ' + controls.ljust(w - 13) + '║', 0)
        row += 1

        # Blank line
        self._draw_text(row, '║' + ' ' * (w - 2) + '║', 0)
        row += 1

        # STATUS section
        listeners = self.bus.listener_count()
        nlm_status = '✓' if listeners else '✗'
        api_status = '✓' if self.bus.api_ready() else '✗'
        status_line = f'LLM: {api_status}  NLM: {nlm_status}  Phase: {self.bus.phase}'
        self._draw_text(row, '║ STATUS: ' + status_line.ljust(w - 11) + '║', 0)
        row += 1

        # Feedback section divider
        self._draw_line(row, w, '╠', '═', '╣')
        row += 1

        # Feedback panel (last 2-3 messages)
        feedback_lines = h - row - 3  # Leave space for input + border
        with self.lock:
            all_msgs = list(self.lines)[-feedback_lines:] if self.lines else []

        for tag, text in all_msgs:
            prefix = {SENT: '> ', OK: '✓ ', BAD: '✗ ', PHASE: '◆ ', INFO: '  '}[tag]
            lines = self._wrap_text(prefix + text, w - 4)
            for line_text in lines[:feedback_lines]:
                feedback = '║ ' + line_text.ljust(w - 3) + '║'
                attr = self._attr(tag)
                try:
                    s.addnstr(row, 0, feedback[:w], w - 1, attr)
                except curses.error:
                    pass
                row += 1
                if row >= h - 3:
                    break
            if row >= h - 3:
                break

        # Pad empty feedback lines
        while row < h - 3:
            self._draw_text(row, '║' + ' ' * (w - 2) + '║', 0)
            row += 1

        # Bottom border
        self._draw_line(row, w, '╚', '═', '╝')
        row += 1

        # Input prompt at bottom
        try:
            s.addnstr(row, 0, '> ' + self.buf, w - 1, curses.A_BOLD)
        except curses.error:
            pass
        s.move(row, min(2 + len(self.buf), w - 1))
        s.refresh()

    def _wrap_text(self, text, width):
        """Wrap text to fit width, return list of lines."""
        return textwrap.wrap(text, width=width) or ['']

    def _draw_line(self, row, w, left, mid, right):
        """Draw a horizontal line."""
        line = left + mid * (w - 2) + right
        self._draw_text(row, line, 0)

    def _draw_text(self, row, text, attr):
        """Draw text safely."""
        try:
            self.stdscr.addnstr(row, 0, text, len(text), attr)
        except curses.error:
            pass

    def key(self, ch):
        """Feed one keypress. Returns a submitted line, or None.
        Arrow keys trigger teleop commands; other keys update the input buffer."""
        if ch in (curses.KEY_BACKSPACE, 127, 8):
            self.buf = self.buf[:-1]
        elif ch in (curses.KEY_ENTER, 10, 13):
            line, self.buf = self.buf.strip(), ''
            return line
        elif ch == curses.KEY_RESIZE:
            pass
        elif ch == curses.KEY_UP:
            self.bus.teleop(0.3, 0.0)  # forward
            self.last_teleop_time = time.time()
        elif ch == curses.KEY_DOWN:
            self.bus.teleop(-0.3, 0.0)  # backward
            self.last_teleop_time = time.time()
        elif ch == curses.KEY_LEFT:
            self.bus.teleop(0.0, 0.5)  # turn left
            self.last_teleop_time = time.time()
        elif ch == curses.KEY_RIGHT:
            self.bus.teleop(0.0, -0.5)  # turn right
            self.last_teleop_time = time.time()
        elif 32 <= ch < 127:
            self.buf += chr(ch)
        return None


def _run(stdscr, bus, screen):
    while True:
        screen.draw()
        try:
            ch = stdscr.get_wch()
        except curses.error:      # nodelay: nothing typed this tick
            curses.napms(60)
            continue
        except KeyboardInterrupt:
            return
        ch = ord(ch) if isinstance(ch, str) else ch
        line = screen.key(ch)
        if line is None:
            continue
        if line in ('/quit', '/exit', '/q'):
            return 'quit'
        # `kill` tears the whole stack down via kill.sh. Run it AFTER curses
        # exits (it SIGKILLs this process too), so signal via the return value.
        if line.strip().lower().lstrip('/') == 'kill':
            return 'kill'
        if not line:
            continue
        # Abort is intercepted before the mission path: it goes straight to the
        # executor service, not through nlm_interface or the LLM.
        if is_abort(line):
            bus.abort()
            continue
        # `reset` puts every Gazebo model back to spawn, straight to the
        # /reset_world service - never a mission, so intercept it here.
        if line.strip().lower().lstrip('/') == 'reset':
            bus.reset()
            continue
        # "save"/"done" finishes an operator phase, straight to the executor.
        # Mapping must be named ("save floor1") so the map isn't left unnamed;
        # collection needs no name (it already knows its map).
        name = parse_finish(line)
        if name is not None:
            if bus.phase == 'mapping':
                if name:
                    bus.save_map(name)
                else:
                    screen.add(BAD, 'name the map to save it, e.g. "save floor1"')
            else:
                bus.done()
            continue
        screen.add(SENT, line)
        if not bus.listener_count():
            screen.add(BAD, 'nlm_interface is not running - command dropped. '
                            'Start it: ros2 launch core_task_controller '
                            'nlm_interface.launch.py')
            continue
        bus.send(line)


def main(args=None):
    """Run the CLI until /quit or Ctrl-C."""
    rclpy.init(args=args)
    holder = {}

    def sink(tag, text):
        screen = holder.get('screen')
        if screen is not None:
            screen.add(tag, text)

    bus = _Bus(sink)

    def spin():
        try:
            rclpy.spin(bus)
        except (rclpy.executors.ExternalShutdownException, RuntimeError):
            pass          # expected: try_shutdown() below unblocks us

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        def boot(stdscr):
            screen = _Screen(stdscr, bus)
            holder['screen'] = screen
            holder['action'] = _run(stdscr, bus, screen)
        curses.wrapper(boot)
    except KeyboardInterrupt:
        pass
    finally:
        # Order matters: shut the context down so spin() returns and the thread
        # is joined before the node goes away. Destroying the node from under a
        # live spin aborts the process with a C++ std::terminate.
        rclpy.try_shutdown()
        thread.join(timeout=2.0)
        bus.destroy_node()

    # `kill` requests the aggressive teardown, but we do NOT run kill.sh here.
    # This process is a child of run.sh's foreground `ros2 run`; kill.sh tears
    # that whole tree down, so running it from inside orphans it and leaves the
    # shell's terminal half-reset (prompt returns while kill.sh is still going).
    # Exit with a sentinel code instead - run.sh sees 42 and runs kill.sh itself,
    # as the process that actually owns the terminal, for a clean hand-back.
    if holder.get('action') == 'kill':
        raise SystemExit(42)


if __name__ == '__main__':
    main()
