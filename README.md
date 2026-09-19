# zcfan | [![Tests](https://img.shields.io/github/actions/workflow/status/cdown/zcfan/ci.yml?branch=master)](https://github.com/cdown/zcfan/actions?query=branch%3Amaster)

Zero-configuration fan control daemon for ThinkPads.

## Features

- Extremely small, simple, and easy to understand code
- Sensible out of the box, configuration is optional (see "usage" below)
- Strong focus on stopping the fan as soon as safe to do so, without inducing
  throttling
- Automatic temperature- and time-based hysteresis: no bouncing between fan
  levels
- Watchdog support
- Minimal resource usage
- No dependencies

## Usage

zcfan reads the temperature inputs exposed by the system's hwmon drivers and
uses their arithmetic average for fan control. When identifiable CPU-core
readings are available, only those readings are averaged. Otherwise, it uses
CPU-labelled readings (or known CPU temperature drivers), and finally falls
back to all readable temperature inputs, which can include die readings when
no CPU sensor can be identified. Unavailable and non-positive readings are
ignored. The set of readable, non-ignored temperature inputs is re-checked
roughly once per second, so hwmon drivers that register after zcfan has started
(for example, coretemp being autoloaded by udev during boot) are picked up
automatically. The fan-control temperature is the maximum of the core average
and the ACPI/EC sensor reading (the same reading the firmware's critical
shutdown trip reacts to). Die/package sensor readings are deliberately
excluded: they spike hotter and faster than the trip sensor. The ACPI/EC
reading normally only raises the fan (engagement and the 95C panic, which
also holds maximum): level reductions follow the core average, except when
no genuine core average exists (then the fan-control temperature decides)
and in the 95C panic band (then no reduction is considered at all: maximum
is held outright). zcfan has the following default fan states:

| Config name     | thinkpad_acpi fan level           | Default trip temperature (C) | Default debounce (s) |
|-----------------|-----------------------------------|------------------------------|----------------------|
| max_temp        | full-speed (or 7 if unsupported)  | 90                           | 10                   |
| med_temp        | 4                                 | 80                           | 30                   |
| low_temp        | 1                                 | 70                           | 60                   |

If no trip temperature is reached (and the reading is below the 95C panic
band), the fan will be turned off.

The fan will also only be reduced once the core average is at least 20C below
the trip temperature for the current fan state. In the 95C panic band no
reduction is considered at all (maximum is held outright), and when no
genuine core average is available the fan-control temperature decides
instead (see above). This can be tuned with the config parameter
`temp_hysteresis`.

To override these defaults, you can place a file at `/etc/zcfan.conf` with
updated trip temperatures in degrees celsius and/or fan levels. As an example:

    max_temp 85
    med_temp 70
    low_temp 55
    temp_hysteresis 20

    max_level full-speed
    med_level 4
    low_level 1

The number of consecutive seconds the fan-control temperature must stay above a
level's trip temperature before that level is engaged can be set per level with
the `max_debounce_secs`, `med_debounce_secs`, and `low_debounce_secs` config
parameters. A value of `0` or `1` engages the level immediately. For example:

    max_debounce_secs 1
    med_debounce_secs 3
    low_debounce_secs 5

The debounce is bypassed at 95C and above: the maximum fan level is engaged
immediately and held while the temperature stays there, to stay clear of the
firmware's critical shutdown temperature.

### Ignoring sensors

If you have a faulty sensor, or a sensor that you otherwise want to ignore, you
can ignore it using the `ignore_sensor` directive in the config file. For
example:

    % grep . /sys/class/hwmon/*/name
    /sys/class/hwmon/hwmon0/name:AC
    /sys/class/hwmon/hwmon1/name:acpitz
    /sys/class/hwmon/hwmon2/name:BAT0
    /sys/class/hwmon/hwmon3/name:nvme
    /sys/class/hwmon/hwmon4/name:coretemp

If you wanted to ignore the nvme and BAT0 monitors, you'd add to your config
file:

    ignore_sensor nvme
    ignore_sensor BAT0

### Hysteresis

We will only reduce the fan level again once:

1. The core average is at least `temp_hysteresis` Celsius (default 20C)
   below the trip point -- except in the 95C panic band and when the core
   average is unreadable, where the fan-control temperature decides (see
   above), and
2. At least 3 seconds have elapsed since the initial trip.

This avoids unnecessary fluctuations in fan speed.

### Debounce

Conversely, before we engage a *higher* fan level, the fan-control temperature
must remain above that level's trip point for a number of consecutive seconds
set by
`low_debounce_secs`, `med_debounce_secs`, and `max_debounce_secs` (defaults 60,
30, and 10). This prevents brief temperature spikes from needlessly spinning the
fans up. A value of `0` or `1` engages the level immediately.

## Comparison with thinkfan

I wrote zcfan because I found thinkfan's configuration and code complexity too
much for my tastes. Use whichever suits your needs.

## Compilation

Run `make`.

## Installation

1. Compile zcfan or install from the [AUR
   package](https://aur.archlinux.org/packages/zcfan)
2. Load your thinkpad_acpi module with `fan_control=1`
    - At runtime: `rmmod thinkpad_acpi && modprobe thinkpad_acpi fan_control=1`
    - By default: `echo options thinkpad_acpi fan_control=1 > /etc/modprobe.d/99-fancontrol.conf`
3. Run `zcfan` as root (or use the `zcfan` systemd service provided)
4. If you run a laptop which keeps the fan running during suspend, you will also
   want to send `SIGPWR` before sleep and `SIGUSR2` before wakeup to avoid
   that. For systemd users, enable the `zcfan-sleep.service` and
   `zcfan-resume.service` units to do that automatically.

## Disclaimer

While the author uses this on their own machine, obviously there is no warranty
whatsoever.
