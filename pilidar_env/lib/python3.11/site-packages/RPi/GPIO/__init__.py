# Copyright (c) 2022-2023 Dave Jones <dave@waveform.org.uk>
#
# SPDX-License-Identifier: MIT

import os
import sys
import struct
import warnings
from time import sleep
from pathlib import Path
from threading import Event
from weakref import WeakValueDictionary

import lgpio

try:
    # Patch several constants which changed incompatibly between 0.1.6.0
    # (jammy) and 0.2.0.0 (kinetic)
    lgpio.SET_PULL_NONE
except AttributeError:
    lgpio.SET_PULL_NONE = lgpio.SET_BIAS_DISABLE
    lgpio.SET_PULL_UP = lgpio.SET_BIAS_PULL_UP
    lgpio.SET_PULL_DOWN = lgpio.SET_BIAS_PULL_DOWN

# This is *not* the version of rpi-lgpio, but is the version of RPi.GPIO we
# seek to emulate
VERSION = '0.7.2'

UNKNOWN = -1
BOARD = 10
BCM = 11

PUD_OFF = 20
PUD_DOWN = 21
PUD_UP = 22

OUT = 0
IN = 1
LOW = 0
HIGH = 1

RISING = 31
FALLING = 32
BOTH = 33

SERIAL = 40
SPI = 41
I2C = 42
HARD_PWM = 43


# Note the nuance of the early boards (in which GPIO0/1 and GPIO2/3 were
# switched) is not represented here. This library (currently) has no intention
# of supporting the early Pi boards. As such, this mapping only represents the
# Pi Model B+ onwards
_BOARD_MAP = {
    3: 2, 5: 3, 7: 4, 8: 14, 10: 15, 11: 17, 12: 18, 13: 27, 15: 22, 16: 23,
    18: 24, 19: 10, 21: 9, 22: 25, 23: 11, 24: 8, 26: 7, 29: 5, 31: 6, 32: 12,
    33: 13, 35: 19, 36: 16, 37: 26, 38: 20, 40: 21,
}
_BCM_MAP = {channel: gpio for (gpio, channel) in _BOARD_MAP.items()}

# LG mode constants
_LG_INPUT = 0x100
_LG_OUTPUT = 0x200
_LG_ALERT = 0x400
_LG_GROUP = 0x800
_LG_MODES = (_LG_INPUT | _LG_OUTPUT | _LG_ALERT | _LG_GROUP)
_LG_PULL_UP = 0x20
_LG_PULL_DOWN = 0x40
_LG_PULL_NONE = 0x80
_LG_PULLS = (_LG_PULL_UP | _LG_PULL_DOWN | _LG_PULL_NONE)

_mode = UNKNOWN
_chip = None
_warnings = True


# Mapping of GPIO number to _Alert instances
_alerts = {}
class _Alert:
    """
    A trivial class encapsulating a single GPIO set for alerts. Stores the
    edge (which we override with the gpiochip API2 result, if available), the
    bouncetime (which we can't get from anywhere else), the default tally
    callback, and the list of user callbacks.
    """
    __slots__ = (
        'gpio', '_edge', 'bouncetime', 'callbacks', '_detected', '_callback')

    def __init__(self, gpio, edge, bouncetime=None):
        self.gpio = gpio
        self._edge = edge
        self.bouncetime = bouncetime
        self.callbacks = []
        self._detected = False
        if bouncetime is not None:
            _check(lgpio.gpio_set_debounce_micros(
                _chip, gpio, bouncetime * 1000))
        self._callback = lgpio.callback(_chip, gpio, func=self._call)

    def __repr__(self):
        return f'_Alert({self.gpio}, {self.edge}, {self.bouncetime})'

    def close(self):
        self._callback.cancel()

    def _call(self, chip, gpio, level, timestamp):
        if level == 2:
            # Watchdog timeout; this *shouldn't* happen as we never use this
            # part of lgpio but if there's something else messing with the API
            # other than this shim it's a possibility
            return
        self._detected = True
        for cb in self.callbacks:
            try:
                cb(_from_gpio(gpio))
            except Exception as exc:
                # Bug compatibility: this is how RPi.GPIO operates
                print(exc, file=sys.stderr)

    @property
    def edge(self):
        # Attempt to determine the edges for this alert from gpiochip API2. If
        # this results in no edges, we're on gpiochip API1, so use the stored
        # value.
        mode = (lgpio.gpio_get_mode(_chip, self.gpio) >> 17) & 3
        try:
            return {
                1: lgpio.RISING_EDGE,
                2: lgpio.FALLING_EDGE,
                3: lgpio.BOTH_EDGES,
            }[mode]
        except KeyError:
            return self._edge

    @property
    def detected(self):
        if self._detected:
            self._detected = False
            return True
        return False


_pwms = WeakValueDictionary()
class PWM:
    """
    Initializes and controls software-based PWM (Pulse Width Modulation) on the
    specified *channel* at *frequency* (in Hz).

    Call :meth:`start` and :meth:`stop` to generate and stop the actual
    output respectively. :meth:`ChangeFrequency` and :meth:`ChangeDutyCycle`
    can also be used to control the output.

    .. note::

        Letting the :class:`PWM` object go out of scope (and be garbage
        collected) will implicitly stop the PWM.

    .. _PWM: https://en.wikipedia.org/wiki/Pulse-width-modulation
    """
    __slots__ = ('_gpio', '_frequency', '_dc', '_running', '__weakref__')

    def __init__(self, channel, frequency):
        self._gpio = _to_gpio(channel)
        if self._gpio in _pwms:
            raise RuntimeError(
                'A PWM object already exists for this GPIO channel')
        _check_output(lgpio.gpio_get_mode(_chip, self._gpio))
        _pwms[self._gpio] = self
        self._frequency = None
        self._dc = None
        self._running = False
        self.ChangeFrequency(frequency)
        if self._frequency <= 0.0:
            raise ValueError('frequency must be greater than 0.0')

    def __del__(self):
        self.stop()

    def start(self, dc):
        """
        Starts outputting a wave on the assigned pin with a duty-cycle (which
        must be between 0 and 100) given by *dc*.

        :param float dc:
            The duty-cycle (the percentage of time the pin is "on")
        """
        self.ChangeDutyCycle(dc)
        self._running = True
        lgpio.tx_pwm(_chip, self._gpio, self._frequency, dc)

    def stop(self):
        """
        Stops outputting a wave on the assigned pin, and sets the pin's state
        to off.
        """
        # We do not care about errors in stop; __del__ methods (from which this
        # is called) should generally avoid exceptions but moreover, it should
        # be idempotent on outputs
        try:
            lgpio.tx_pwm(_chip, self._gpio, 0, 0)
        except lgpio.error:
            pass
        lgpio.gpio_write(_chip, self._gpio, 0)
        self._running = False

    def ChangeDutyCycle(self, dc):
        """
        Changes the duty cycle (percentage of the time that the pin is "on")
        to *dc*.
        """
        self._dc = float(dc)
        if not 0 <= self._dc <= 100:
            raise ValueError('dutycycle must have a value from 0.0 to 100.0')
        if self._running:
            lgpio.tx_pwm(_chip, self._gpio, self._frequency, self._dc)

    def ChangeFrequency(self, frequency):
        """
        Changes the *frequency* of rising edges output by the pin.
        """
        self._frequency = float(frequency)
        if self._frequency <= 0.0:
            raise ValueError('frequency must be greater than 0.0')
        if self._running:
            lgpio.tx_pwm(_chip, self._gpio, self._frequency, self._dc)


def _check(result):
    """
    Many lgpio functions return <0 on error; this simple function just converts
    any *result* less than zero to the appropriate :exc:`RuntimeError` message
    and passes non-negative results back to the caller.
    """
    if result < 0:
        raise RuntimeError(lgpio.error_text(result))
    return result


def _check_input(mode,
                 msg='You must setup() the GPIO channel as an input first'):
    """
    Raises :exc:`RuntimeError` if *mode* (as returned by
    :func:`lgpio.gpio_get_mode`) does not indicate that the GPIO is configured
    for "Input" or "Alert".
    """
    if not mode & (_LG_INPUT | _LG_ALERT):
        raise RuntimeError(msg)


def _check_output(mode,
                  msg='You must setup() the GPIO channel as an output first'):
    """
    Raises :exc:`RuntimeError` if *mode* (as returned by
    :func:`lgpio.gpio_get_mode`) does not indicate that the GPIO is configured
    for "Output".
    """
    if not mode & _LG_OUTPUT:
        raise RuntimeError(msg)


def _check_edge(edge):
    """
    Checks *edge* is a valid value.
    """
    if edge not in (FALLING, RISING, BOTH):
        raise ValueError('The edge must be set to RISING, FALLING or BOTH')


def _check_bounce(bouncetime):
    """
    Checks *bouncetime* is :data:`None` or a positive value.
    """
    # The value -666 is special in RPi.GPIO, and is used internally as the
    # default of no bouncetime; we convert this to None (which is lgpio's
    # Python binding's default)
    if bouncetime == -666:
        bouncetime = None
    if bouncetime is not None and bouncetime <= 0:
        raise ValueError('Bouncetime must be greater than 0')
    return bouncetime


def _get_alert(gpio, mode, edge, bouncetime):
    """
    Returns the :class:`_Alert` object for the specified *gpio* (which has the
    specifed *mode*, as returned by :func:`lgpio.gpio_get_mode`), but only if
    it has compatible *edge* and *bouncetime* settings. If no alerts are set,
    :exc:`KeyError` is raised. If alerts are set, but with incompatible *edge*
    or *bouncetime* values, :exc:`RuntimeError` is raised.
    """
    if not mode & _LG_ALERT:
        raise KeyError(gpio)
    alert = _alerts[gpio]
    if alert.edge != edge or alert.bouncetime != bouncetime:
        raise RuntimeError(
            'Conflicting edge detection already enabled for this GPIO '
            'channel')
    return alert


def _set_alert(gpio, mode, edge, bouncetime):
    """
    Set up alerts on a *gpio*. The *mode* is the current GPIO mode as returned
    by :func:`lgpio.gpio_get_mode`. The *edge* is the desired edge detection,
    and *bouncetime* the desired debounce delay.
    """
    _check(lgpio.gpio_claim_alert(_chip, gpio, {
        RISING:  lgpio.RISING_EDGE,
        FALLING: lgpio.FALLING_EDGE,
        BOTH:    lgpio.BOTH_EDGES,
    }[edge], mode & _LG_PULLS))
    if bouncetime is not None:
        _check(lgpio.gpio_set_debounce_micros(
            _chip, gpio, bouncetime * 1000))
    alert = _Alert(gpio, edge, bouncetime)
    _alerts[gpio] = alert
    return alert


def _unset_alert(gpio):
    """
    Remove alerts on *gpio*. This doesn't actually remove the claimed alert
    status (in lgpio parlance), but it does cancel all associated callbacks and
    remove the relevant :class:`_Alert` instance.
    """
    try:
        alert = _alerts.pop(gpio)
    except KeyError:
        pass
    else:
        alert.close()


def _retry(func, *args, _count=3, _delay=0.001, **kwargs):
    """
    Under certain circumstances (usually multiple concurrent processes
    accessing the same GPIO device), GPIO functions can return "GPIO_BUSY". In
    this case the operation should simply be retried after a delay.
    """
    for i in range(_count):
        result = func(*args, **kwargs)
        if result != lgpio.GPIO_BUSY:
            return _check(result)
        sleep(_delay)
    raise RuntimeError(lgpio.error_text(lgpio.GPIO_BUSY))


def _to_gpio(channel):
    """
    Converts *channel* to a GPIO number, according to the globally set
    :data:`_mode`.
    """
    if _mode == UNKNOWN:
        raise RuntimeError(
            'Please set pin numbering mode using GPIO.setmode(GPIO.BOARD) or '
            'GPIO.setmode(GPIO.BCM)')
    elif _mode == BCM:
        if not 0 <= channel < 54:
            raise ValueError('The channel sent is invalid on a Raspberry Pi')
        return channel
    elif _mode == BOARD:
        try:
            return _BOARD_MAP[channel]
        except KeyError:
            raise ValueError('The channel sent is invalid on a Raspberry Pi')
    else:
        assert False, 'Invalid channel mode'


def _from_gpio(gpio):
    """
    Converts *gpio* to a channel number, according to the globally set
    :data:`_mode`.
    """
    if _mode == BCM:
        return gpio
    elif _mode == BOARD:
        return _BCM_MAP[gpio]
    else:
        raise RuntimeError(
            'Please set pin numbering mode using GPIO.setmode(GPIO.BOARD) or '
            'GPIO.setmode(GPIO.BCM)')


def _gpio_list(chanlist):
    """
    Convert *chanlist* which may be an iterable, or an int, to a tuple of
    integers
    """
    try:
        return tuple(_to_gpio(int(channel)) for channel in chanlist)
    except TypeError:
        try:
            return (_to_gpio(int(chanlist)),)
        except TypeError:
            raise ValueError(
                'Channel must be an integer or list/tuple of integers')


def _in_use(gpio):
    """
    Returns :data:`True` if the GPIO has been "claimed" by lgpio. lgpio mode
    bits (256, 512, 1024, 2048) are only set if the calling process owns the
    GPIO.
    """
    return bool(_check(lgpio.gpio_get_mode(_chip, gpio)) & _LG_MODES)


def _get_rpi_info():
    """
    Queries the device-tree for the board revision, throwing :exc:`RuntimeError`
    if it cannot be found, then returns a :class:`dict` containing information
    about the board.
    """
    try:
        revision = int(os.environ['RPI_LGPIO_REVISION'], base=16)
    except KeyError:
        try:
            with open('/proc/device-tree/system/linux,revision', 'rb') as f:
                revision = struct.unpack('>I', f.read(4))[0]
            if not revision:
                raise OSError()
        except OSError:
            raise RuntimeError('This module can only be run on a Raspberry Pi!')
    if not (revision >> 23 & 0x1):
        raise NotImplementedError(
            'This module does not understand old-style revision codes')
    return {
        'P1_REVISION': {
            0x00: 2,
            0x01: 2,
            0x06: 0,
            0x0a: 0,
            0x10: 0,
            0x14: 0,
        }.get(revision >> 4 & 0xff, 3),
        'REVISION': hex(revision)[2:],
        'TYPE': {
            0x00: 'Model A',
            0x01: 'Model B',
            0x02: 'Model A+',
            0x03: 'Model B+',
            0x04: 'Pi 2 Model B',
            0x05: 'Alpha',
            0x06: 'Compute Module 1',
            0x08: 'Pi 3 Model B',
            0x09: 'Zero',
            0x0a: 'Compute Module 3',
            0x0c: 'Zero W',
            0x0d: 'Pi 3 Model B+',
            0x0e: 'Pi 3 Model A+',
            0x10: 'Compute Module 3+',
            0x11: 'Pi 4 Model B',
            0x12: 'Zero 2 W',
            0x13: 'Pi 400',
            0x14: 'Compute Module 4',
            0x17: 'Pi 5 Model B',
        }.get(revision >> 4 & 0xff, 'Unknown'),
        'MANUFACTURER': {
            0: 'Sony UK',
            1: 'Egoman',
            2: 'Embest',
            3: 'Sony Japan',
            4: 'Embest',
            5: 'Stadium',
        }.get(revision >> 16 & 0xf, 'Unknown'),
        'PROCESSOR': {
            0: 'BCM2835',
            1: 'BCM2836',
            2: 'BCM2837',
            3: 'BCM2711',
            4: 'BCM2712',
        }.get(revision >> 12 & 0xf, 'Unknown'),
        'RAM': {
            0: '256M',
            1: '512M',
            2: '1GB',
            3: '2GB',
            4: '4GB',
            5: '8GB',
            6: '16GB',
        }.get(revision >> 20 & 0x7, 'Unknown'),
    }


def _get_gpiochip_num():
    """
    Determines the number of the GPIO chip device to access.

    If :envvar:`RPI_LGPIO_CHIP` is found in the environment, it will be used.
    Otherwise, the routine will attempt to query sysfs to find a GPIO chip
    device with a driver known to be used for userspace GPIO access. If none
    can be found, returns 0 as a fallback.
    """
    try:
        return int(os.environ['RPI_LGPIO_CHIP'])
    except KeyError:
        # The following are the driver names used for the GPIO chip devices
        # intended for userspace control
        user_gpio_drivers = frozenset((
            'raspberrypi,rp1-gpio',
            'raspberrypi,bcm2835-gpio',
            'raspberrypi,bcm2711-gpio',
        ))
        for dev in Path('/sys/bus/gpio/devices').glob('gpiochip*'):
            compatible = (dev / 'of_node/compatible')
            try:
                drivers = set(compatible.read_text().split('\0'))
            except FileNotFoundError:
                continue
            if drivers & user_gpio_drivers:
                return int(dev.name[len('gpiochip'):])
    return 0


def getmode():
    """
    Get the numbering mode used for the pins on the board. Returns
    :data:`BOARD`, :data:`BCM` or :data:`None`.
    """

    if _mode == UNKNOWN:
        return None
    else:
        return _mode


def setmode(new_mode):
    """
    Set up the numbering mode to use for the pins on the board. The options
    for *new_mode* are:

    * :data:`BOARD` - Use Raspberry Pi board numbers
    * :data:`BCM` - Use Broadcom GPIO 00..nn numbers

    If a numbering mode has already been set, and *new_mode* is not the same
    as the result of :func:`getmode`, a :exc:`ValueError` is raised.

    :param int new_mode:
        The new numbering mode to apply
    """
    global _mode, _chip

    if _mode != UNKNOWN and new_mode != _mode:
        raise ValueError('A different mode has already been set!')
    if new_mode not in (BOARD, BCM):
        raise ValueError('An invalid mode was passed to setmode()')

    if _chip is None:
        _chip = _check(lgpio.gpiochip_open(_get_gpiochip_num()))
    _mode = new_mode


def setwarnings(value):
    """
    Enable or disable warning messages. These are mostly produced when calling
    :func:`setup` or :func:`cleanup` to change channel modes.
    """
    global _warnings
    _warnings = bool(value)


def gpio_function(channel):
    """
    Return the current GPIO function (:data:`IN`, :data:`OUT`,
    :data:`HARD_PWM`, :data:`SERIAL`, :data:`I2C`, :data:`SPI`) for the
    specified *channel*.

    .. note::

        This function will only return :data:`IN` or :data:`OUT` under
        rpi-lgpio as the underlying kernel device cannot report the alt-mode of
        GPIO pins.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode`
    """
    gpio = _to_gpio(channel)
    mode = _check(lgpio.gpio_get_mode(_chip, gpio))
    if mode & 0x2:
        return OUT
    else:
        return IN


def cleanup(chanlist=None):
    """
    Reset the specified GPIO channels (or all channels if none are specified)
    to INPUT with no pull-up / pull-down and no event detection.

    :type chanlist: list or tuple or int or None
    :param chanlist:
        The channel, or channels to clean up
    """
    global _chip, _mode
    if _chip is None:
        return

    # If we're cleaning up everything we need to close the chip handle too,
    # and reset the GPIO mode. But first...
    close = chanlist is None
    if chanlist is None:
        # Bug compatibility: it's awfully tempting to just re-initialize here,
        # but that doesn't reset pins to inputs, and users may be relying upon
        # this side-effect
        result, gpios, *tail = lgpio.gpio_get_chip_info(_chip)
        _check(result)
        chanlist = [gpio for gpio in range(gpios) if _in_use(gpio)]
    else:
        chanlist = _gpio_list(chanlist)

    if chanlist:
        for gpio in chanlist:
            # As this is cleanup we ignore all errors (no _check calls); if we
            # didn't own the GPIO, we don't care
            _unset_alert(gpio)
            lgpio.gpio_claim_input(_chip, gpio, lgpio.SET_PULL_NONE)
            lgpio.gpio_free(_chip, gpio)
    elif _warnings:
        warnings.warn(Warning(
            'No channels have been set up yet - nothing to clean up!  Try '
            'cleaning up at the end of your program instead!'))

    if close:
        lgpio.gpiochip_close(_chip)
        _chip = None
        _mode = UNKNOWN
        assert not _alerts


def setup(chanlist, direction, pull_up_down=PUD_OFF, initial=None):
    """
    Set up a GPIO channel or iterable of channels with a direction and
    (optionally, for inputs) pull/up down control, or (optionally, for outputs)
    and initial state.

    The GPIOs to affect are listed in *chanlist* which may be any iterable. The
    *direction* is either :data:`IN` or :data:`OUT`.

    If *direction* is :data:`IN`, then *pull_up_down* may specify one of the
    values :data:`PUD_UP` to set the internal pull-up resistor,
    :data:`PUD_DOWN` to set the internal pull-down resistor, or the default
    :data:`PUD_OFF` which disables the internal pulls.

    If *direction* is :data:`OUT`, then *initial* may specify zero or one to
    indicate the initial state of the output.

    :type chanlist: list or tuple or int
    :param chanlist:
        The list of GPIO channels to setup

    :param int direction:
        Whether the channels should act as inputs or outputs

    :type pull_up_down: int or None
    :param pull_up_down:
        The internal pull resistor (if any) to enable for inputs

    :type initial: bool or int or None
    :param initial:
        The initial state of an output
    """
    if direction == OUT:
        if pull_up_down != PUD_OFF:
            raise ValueError('pull_up_down parameter is not valid for outputs')
        if initial is not None:
            initial = bool(initial)
    elif direction == IN:
        if initial is not None:
            raise ValueError('initial parameter is not valid for inputs')
        if pull_up_down not in (PUD_UP, PUD_DOWN, PUD_OFF):
            raise ValueError(
                'Invalid value for pull_up_down - should be either PUD_OFF, '
                'PUD_UP or PUD_DOWN')
    else:
        raise ValueError('An invalid direction was passed to setup()')

    for gpio in _gpio_list(chanlist):
        # We don't bother with warnings about GPIOs already in use here because
        # if we try to *use* a GPIO already in use, things are going to blow up
        # shortly anyway. We do deal with the pull-up warning, but only for
        # GPIO2 and GPIO3 because we're not supporting the original RPi so we
        # don't need to worry about the GPIO0 and GPIO1 discrepancy
        if _warnings and gpio in (2, 3) and pull_up_down in (PUD_UP, PUD_DOWN):
            warnings.warn(Warning(
                'A physical pull up resistor is fitted on this channel!'))
        if direction == IN:
            # This gpio_free may seem redundant, but is required when changing
            # the line-flags of an already acquired input line
            try:
                lgpio.gpio_free(_chip, gpio)
            except lgpio.error:
                pass
            _check(lgpio.gpio_claim_input(_chip, gpio, {
                PUD_OFF:  lgpio.SET_PULL_NONE,
                PUD_DOWN: lgpio.SET_PULL_DOWN,
                PUD_UP:   lgpio.SET_PULL_UP,
            }[pull_up_down]))
        elif direction == OUT:
            _unset_alert(gpio)
            if initial is None:
                initial = _check(lgpio.gpio_read(_chip, gpio))
            _check(lgpio.gpio_claim_output(
                _chip, gpio, initial, lgpio.SET_PULL_NONE))
        else:
            assert False, 'Invalid direction'


def input(channel):
    """
    Input from a GPIO *channel*. Returns 1 or 0.

    This can also be called on a GPIO output, in which case the value
    returned will be the last state set on the GPIO.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode`
    """
    gpio = _to_gpio(channel)
    if not _in_use(gpio):
        raise RuntimeError('You must setup() the GPIO channel first')
    return _check(lgpio.gpio_read(_chip, gpio))


def output(channel, value):
    """
    Output to a GPIO *channel* or list of channels. The *value* can be the
    integer :data:`LOW` or :data:`HIGH`, or a list of integers.

    If a list of channels is specified, with a single integer for the *value*
    then it is applied to all channels. Otherwise, the length of the two lists
    must match.

    :type channel: list or tuple or int
    :param channel:
        The GPIO channel, or list of GPIO channels to output to

    :type value: list or tuple or int
    :param value:
        The value, or list of values to output
    """
    gpios = _gpio_list(channel)
    try:
        values = tuple(bool(item) for item in value)
    except TypeError:
        try:
            values = (bool(value),)
        except TypeError:
            raise ValueError(
                'Value must be an integer/boolean or list/tuple of '
                'integers/booleans')
    if len(gpios) != len(values):
        if len(gpios) > 1 and len(values) == 1:
            values = values * len(gpios)
        else:
            raise RuntimeError('Number of channels != number of values')
    for gpio, value in zip(gpios, values):
        mode = lgpio.gpio_get_mode(_chip, gpio)
        _check_output(mode, 'The GPIO channel has not been set up as an OUTPUT')
        _check(lgpio.gpio_write(_chip, gpio, value))


def wait_for_edge(channel, edge, bouncetime=None, timeout=None):
    """
    Wait for an *edge* on the specified *channel*. Returns *channel* or
    :data:`None` if *timeout* elapses before the specified edge occurs.

    .. note::

        Debounce works significantly differently in rpi-lgpio than it does
        in rpi-gpio; please see :ref:`debounce` for more information on the
        differences.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode` to
        watch for changes

    :param int edge:
        One of the constants :data:`RISING`, :data:`FALLING`, or :data:`BOTH`

    :type bouncetime: int or None
    :param bouncetime:
        Time (in ms) used to debounce signals

    :type timeout: int or None
    :param timeout:
        Maximum time (in ms) to wait for the edge
    """
    gpio = _to_gpio(channel)
    mode = _check(lgpio.gpio_get_mode(_chip, gpio))
    _check_input(mode)
    _check_edge(edge)
    bouncetime = _check_bounce(bouncetime)
    if timeout is not None and timeout <= 0:
        raise ValueError('Timeout must be greater than 0')

    try:
        alert = _get_alert(gpio, mode, edge, bouncetime)
    except KeyError:
        unset = True
        alert = _set_alert(gpio, mode, edge, bouncetime)
    else:
        unset = False
        # Bug compatibility: this is how RPi.GPIO operates
        if alert.callbacks:
            raise RuntimeError(
                'Conflicting edge detection already enabled for this GPIO '
                'channel')
    evt = Event()
    alert.callbacks.append(lambda i: evt.set())
    if timeout is not None:
        timeout /= 1000
    if evt.wait(timeout):
        result = channel
    else:
        result = None
    if unset:
        _unset_alert(gpio)
    return result


def add_event_detect(channel, edge, callback=None, bouncetime=None):
    """
    Start background *edge* detection on the specified GPIO *channel*.

    If *callback* is specified, it must be a callable that will be executed
    when the specified *edge* is seen on the GPIO *channel*. The callable must
    accept a single parameter: the channel on which the edge was detected.

    .. note::

        Debounce works significantly differently in rpi-lgpio than it does
        in rpi-gpio; please see :ref:`debounce` for more information on the
        differences.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode` to
        watch for changes

    :param int edge:
        One of the constants :data:`RISING`, :data:`FALLING`, or :data:`BOTH`

    :type callback: callable or None
    :param callback:
        The callback to run when an edge is detected; must take a single
        integer parameter of the channel on which the edge was detected

    :type bouncetime: int or None
    :param bouncetime:
        Time (in ms) used to debounce signals
    """
    if callback is not None and not callable(callback):
        raise TypeError('Parameter must be callable')
    gpio = _to_gpio(channel)
    mode = _check(lgpio.gpio_get_mode(_chip, gpio))
    _check_input(mode)
    _check_edge(edge)
    bouncetime = _check_bounce(bouncetime)
    try:
        alert = _get_alert(gpio, mode, edge, bouncetime)
    except KeyError:
        alert = _set_alert(gpio, mode, edge, bouncetime)

    if callback is not None:
        alert.callbacks.append(callback)


def add_event_callback(channel, callback):
    """
    Add a *callback* to the specified GPIO *channel* which must already have
    been set for background edge detection with :func:`add_event_detect`.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode` to
        watch for changes

    :param callback:
        The callback to run when an edge is detected; must take a single
        integer parameter of the channel on which the edge was detected
    """
    if not callable(callback):
        raise TypeError('Parameter must be callable')
    gpio = _to_gpio(channel)
    mode = _check(lgpio.gpio_get_mode(_chip, gpio))
    _check_input(mode)
    try:
        alert = _alerts[gpio]
    except KeyError:
        raise RuntimeError(
            'Add event detection using add_event_detect first before adding '
            'a callback')
    else:
        alert.callbacks.append(callback)


def remove_event_detect(channel):
    """
    Remove background event detection for the specified *channel*.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode` to
        watch for changes
    """
    _unset_alert(_to_gpio(channel))


def event_detected(channel):
    """
    Returns :data:`True` if an edge has occurred on the specified *channel*
    since the last query of the channel (if any). Querying this will also
    reset the internal edge detected flag for this channel.

    The *channel* must previously have had edge detection enabled with
    :func:`add_event_detect`.

    :param int channel:
        The board pin number or BCM number depending on :func:`setmode`
    """
    try:
        return _alerts[_to_gpio(channel)].detected
    except KeyError:
        return False


RPI_INFO = _get_rpi_info()
RPI_REVISION = RPI_INFO['P1_REVISION']
