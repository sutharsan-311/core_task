"""Unit tests for nlm_cli's pure screen logic.

The rendering needs a real terminal, but the keypress handling and the
transcript formatting do not. `_Screen` is built with object.__new__ to skip
the curses setup in __init__, since none of it is used here.
"""
import curses
import threading
from collections import deque

import pytest

from core_task_controller.nlm_cli import (
    _Screen, is_abort, parse_finish, BAD, INFO, OK, PHASE, SENT,
)


def test_parse_finish_extracts_map_name():
    """'save <name>' yields the name; bare finish verbs yield ''."""
    assert parse_finish('save floor1') == 'floor1'
    assert parse_finish('/save floor 2') == 'floor 2'   # slash + multi-word name
    assert parse_finish('finished lab') == 'lab'
    assert parse_finish('save') == ''
    assert parse_finish('done') == ''
    assert parse_finish('done collecting') == ''         # multi-word phrase, no name


def test_parse_finish_ignores_non_finish():
    """A mission or empty line is not a finish command."""
    assert parse_finish('patrol the warehouse twice') is None
    assert parse_finish('') is None


@pytest.fixture
def screen():
    """Build a _Screen with its curses half left uninitialised."""
    s = object.__new__(_Screen)
    s.lines = deque(maxlen=500)
    s.lock = threading.Lock()
    s.buf = ''
    s.colour = {}
    return s


def type_in(screen, text):
    """Feed each character of `text` through key()."""
    for ch in text:
        screen.key(ord(ch))


def test_typing_accumulates(screen):
    """Printable characters build the input buffer."""
    type_in(screen, 'patrol twice')
    assert screen.buf == 'patrol twice'


def test_enter_submits_and_clears(screen):
    """Enter returns the line and resets the buffer."""
    type_in(screen, 'patrol twice')
    assert screen.key(10) == 'patrol twice'
    assert screen.buf == ''


def test_backspace_deletes(screen):
    """Backspace removes the last character."""
    type_in(screen, 'abc')
    screen.key(curses.KEY_BACKSPACE)
    assert screen.buf == 'ab'


def test_backspace_on_empty_is_safe(screen):
    """Backspace on an empty buffer does not raise."""
    screen.key(curses.KEY_BACKSPACE)
    assert screen.buf == ''


def test_enter_on_empty_returns_empty(screen):
    """Enter with nothing typed submits an empty string, not None."""
    assert screen.key(10) == ''


def test_typing_returns_none(screen):
    """Only Enter submits; ordinary keys return None."""
    assert screen.key(ord('a')) is None


def test_every_tag_renders(screen):
    """_wrapped covers all tags; a missing one would KeyError at runtime."""
    for tag in (SENT, OK, BAD, PHASE, INFO):
        screen.add(tag, 'x')
    rendered = screen._wrapped(60)
    assert len(rendered) == 5


def test_long_line_wraps_and_indents(screen):
    """A long message wraps, and continuation lines align under the first."""
    screen.add(BAD, 'word ' * 40)
    rendered = screen._wrapped(40)
    assert len(rendered) > 1
    assert all(len(text) <= 40 for _, text in rendered)
    assert rendered[1][1].startswith(' ')


def test_transcript_is_bounded(screen):
    """The scrollback cannot grow without limit."""
    for i in range(600):
        screen.add(INFO, str(i))
    assert len(screen.lines) == 500


@pytest.mark.parametrize('text', [
    'abort', 'ABORT', ' abort ', '/abort', 'abort mission',
    'return to start', 'Return to dock', 'go home', 'come home',
    'stop', 'halt', 'cancel',
])
def test_abort_phrases_recognised(text):
    """Every intended stop phrasing is caught locally."""
    assert is_abort(text) is True


@pytest.mark.parametrize('text', [
    'patrol the perimeter twice in warehouse', 'start mapping', 'stop mapping the warehouse',
    'go to the loading dock', 'abort the sandwich order', '',
])
def test_missions_are_not_treated_as_abort(text):
    """Real missions (and near-misses) must not trip the abort path."""
    assert is_abort(text) is False
