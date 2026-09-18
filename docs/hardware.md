# Hardware Interface

Sensor protocol, servo calibration and Raspberry Pi configuration.

---

## 1. Ball position sensing

A 4-wire resistive touchscreen is mounted on the moving platform. The weight of the ball
presses the layers together at the contact point and the screen controller reports the
coordinates over a serial link.

### Wire protocol

| Property | Value |
|---|---|
| Baud rate | 19200 |
| Read size | 10 bytes per sample |
| Sync byte | `0xFF` |
| Payload | 4 bytes, X high, X low, Y high, Y low |
| Encoding | 7 bits per byte |

The parser scans the 10-byte buffer for the last occurrence of `0xFF` and takes the
following four bytes as the coordinate pair. If the sync byte is found at an index greater
than 6, fewer than four payload bytes remain, the sample is discarded and the previous
position is held:

```matlab
j = 0;
for i = 1:10
    if RxData(i) == 255
        j = i + 1;
    end
end

if (j <= 6) && (j > 0)
    raw_x = bitshift(RxData(j),   8) + RxData(j+1);
    raw_y = bitshift(RxData(j+2), 8) + RxData(j+3);
    % remove the 8th bit of each byte; the payload is 7-bit packed
    raw_x = (raw_x - floor(floor(raw_x/128)/2)*128) / 2;
    raw_y = (raw_y - floor(floor(raw_y/128)/2)*128) / 2;
    xb = (raw_x - 20) / 470 * 22.8;   % cm
    yb = (raw_y -  7) / 490 * 30.4;   % cm
end
```

Holding the previous value avoids feeding a zero into the double-integrator plant when the
screen drops a sample, which occurs when the ball is moving quickly.

### Calibration

| Axis | Raw range | Offset | Physical span |
|---|---|---|---|
| X | 20 … 490 | 20 | 22.8 cm |
| Y | 7 … 497 | 7 | 30.4 cm |

Output is in centimetres; the control models convert to metres with an explicit gain block.
The active area is not square, which contributes to the difference between the identified
plant gains of the two axes, see [modelling.md §6](modelling.md).

### Deadband

A ±1.5 mm deadband is applied to the position error in every controller:

```matlab
function y = deadband(ex)
    if abs(ex) < 0.0015
        y = 0;
    else
        y = ex;
    end
end
```

This prevents the quantization noise of the screen from producing continuous small
corrections, which the observer would otherwise differentiate into significant apparent
velocity.

Bring-up models: [`Serial.slx`](../src/hardware/Serial.slx) (read and decode only),
[`Serial_1.slx`](../src/hardware/Serial_1.slx), and
[`stuff.slx`](../src/hardware/stuff.slx) (serial read with sine-driven PWM).

---

## 2. Servo actuation

### Pin assignment

| Servo | Pi GPIO |
|---|---|
| 1 | 12 |
| 2 | 20 |
| 3 | 16 |
| 4 | 17 |
| 5 | 27 |
| 6 | 22 |

All six are configured as PWM at 50 Hz (20 ms period).

### Angle to pulse width

[`Angle2Duty.m`](../src/hardware/Angle2Duty.m) holds six measured calibration tables, one
per servo, each mapping commanded angle to the pulse width in milliseconds that produces
it. Each is fitted with a first-order `polyfit`:

```matlab
M1 = [0,1.14; 6,1.20; 11,1.25; ... 65,1.63];
p1 = polyfit(M1(:,1), M1(:,2), 1);
...
d(1) = dot(p1, [ th(1), 1]);
d(2) = dot(p2, [-th(2), 1]);
d(3) = dot(p3, [ th(3), 1]);
d(4) = dot(p4, [-th(4), 1]);
d(5) = dot(p5, [ th(5), 1]);
d(6) = dot(p6, [-th(6), 1]);
```

The six tables span different angle ranges and pulse-width offsets, servo 1 covers
approximately 1.14 to 1.63 ms while servo 2 covers 1.40 to 2.00 ms for a comparable sweep, so
each requires its own fit. Even-numbered servos take a negated angle because they are
mounted mirrored around the hexagon.

`writePWMDutyCycle` takes a fraction rather than a duration, so the result is divided by
the 20 ms period:

```matlab
duty = Angle2Duty(angles) / 20;
```

### Neutral positions

[`rst2.m`](../src/hardware/rst2.m) holds the measured zero-angle pulse width of each servo,
used to return the platform to level:

| Servo | Neutral (ms) |
|---|---|
| 1 | 1.140 |
| 2 | 1.670 |
| 3 | 1.290 |
| 4 | 1.620 |
| 5 | 1.140 |
| 6 | 1.645 |

[`Reset123.m`](../src/hardware/Reset123.m) sets all six duty cycles to zero.

[`stuff2.m`](../src/hardware/stuff2.m) is an earlier calibration set at 10° increments,
superseded by `Angle2Duty.m`.

---

## 3. Bring-up sequence

```matlab
r = raspi;
freq = 50;
% configurePin / writePWMFrequency for all six pins

angles = inv_ken(0, 0, 23, 0, 0, 0);   % home pose: level plate
duty   = Angle2Duty(angles) / 20;

writePWMDutyCycle(r, Motor1, duty(1));   % ... for all six
```

This is [`Motors.m`](../src/hardware/Motors.m); it establishes the connection and moves the
platform to a known pose, and must be run before deploying a controller.

Test models: [`blink_LED.slx`](../src/hardware/blink_LED.slx) (toolchain check),
[`pwm.slx`](../src/hardware/pwm.slx) (single-channel PWM),
[`test.slx`](../src/hardware/test.slx) (constant pose through the inverse kinematics to all
six PWM channels, open loop).

---

## 4. Code generation configuration

| Setting | Value |
|---|---|
| Target | `ert.tlc` (Embedded Coder) |
| Hardware | ARM Compatible → ARM Cortex |
| RTOS | Linux, multitasking mode |
| Base-rate task priority | 40 |
| External mode | XCP on TCP/IP, port 17725 |
| Signal buffer | 1 000 000 |
| Build directory on target | `/home/pi` |
| Generated with | Simulink Coder 9.3 (R2020a) |

The board address, username and password are stored in the model configuration set and
carry placeholder values (`raspberrypi.local`, `pi`, `CHANGEME`). Set your own under
*Model Settings → Hardware Implementation → Target hardware resources → Board Parameters*
before deploying.

Generated C, build directories and compiled `.elf` binaries are not tracked; they are
produced from the models by the build command. See `.gitignore`.

---

## 5. Timing

| Quantity | Value |
|---|---|
| Control sample time `Ts` | 0.02 s (50 Hz) on hardware |
| PWM frequency | 50 Hz |
| Serial baud rate | 19200 |
| Tilt saturation | ±5° (±4.5° in some PD variants) |

Simulation-only models use sample times from 0.01 to 0.2 s depending on the study; the
deployed models all use 0.02 s.
