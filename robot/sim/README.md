# Firmware simulator (software-in-the-loop)

This runs the **real firmware in `robot/src`, unmodified**, on your PC. It is compiled for the host against a simulated STM32H562 board and driven from Python (`bucky.firmware` in `simulation/backend`).

The drivers, `GPort`, `Compass`, `Sonar`, `Motors`, `main()` and every `RUN_TEST` program run as written. Only the lowest layer is replaced: the vendor SDK and the register-level io drivers.

```
uv run python -m bucky.firmware.build -v                      # build (also happens on first use)
uv run pytest tests/test_firmware_*.py                        # ~100 tests, a few seconds
uv run --extra viewer python -m bucky.firmware.viewer --program testIRPositioning
uv run python scripts/lab.py sweep testDriveForward --kind firmware --grid ball_step_cm=60
```

(Run these from `simulation/backend`. You need g++, cmake and, optionally, ninja. `pybind11` is a dev dependency.)

## How it works

```
robot/src (unmodified)      Compass.cpp  GPort.cpp  Sonar.cpp  Motors.cpp  main.cpp  tests/*.cpp
                               │ inline register code: gpio.h, pwm.h, Timer.h (runs for real)
robot/sim/include           <Arduino.h> <stm32h5xx.h> <HardwareTimer.h> <EEPROM.h> cmsis_sim.h
robot/sim/vendor            PinNames.h pinmap.h Print WString … (copied from stm32duino)
robot/sim/src/io            ADC/DMA, I2C, EXTI/IRQ, Timer, LPTIM, UART, USB serial (replace the .c drivers)
robot/sim/src/devices       LIS2MDL · LSM303AGR · IR board · line board · sonar · encoders · PWM probe
robot/sim/src/core          virtual clock + event queue + interrupt dispatch + lockstep runner
robot/sim/src/bindings      pybind11 module `bucky_fw`
```

### Fake SDK, real drivers
- Registers are RAM structs with the exact CMSIS layout, checked by `static_assert`s.
- `TIM1`, `GPIOA`, `ADC1` and the other peripheral macros point at simulator objects.
- `GPIO ODR/BSRR` are write-hooked, so pin edges (the sonar trigger, a hand-stepped G-port clock) reach the device models immediately.
- `cordic.c` builds its libm fallback.

### Time
- Virtual time only moves inside `millis()`/`micros()` (1 µs each by default, see `sim.set_poll_cost_ns`) and inside `delay*()`.
- Hardware events (mux clock edges, echo edges, reset pulses) are scheduled callbacks.
- Interrupts run at those time hooks, and are deferred while `__disable_irq()` is in effect.

### Programs
`main.cpp` is compiled with `-Dmain=fw_main`. CMake reads `#define RUN_TEST <name>` from `main.cpp` and adds `-D<name>=bucky_sim_run_test`, so `RUN_TEST(ctx)` lands in the simulator, which runs the test Python picked.

| Program | What runs |
|---|---|
| `firmware` | `main()` exactly as built |
| `main_loop` | `main()` with the `RUN_TEST` call skipped |
| `testXxx` | any test defined in `robot/src/tests` |

### Lockstep
- A program runs on its own thread and parks in a time hook when it reaches the deadline Python gave `sim.run_until_ns`. Only one side runs at a time.
- Between steps Python may call side-effect-free getters on `bucky_fw.globals` (for example `compass().getHeading()`, `irPort().readIR()`). Anything that reads the clock raises `SimMisuse`.
- A loop with no time hook (`while (true) {}`) is caught by a wall-clock watchdog (`FirmwareHang`). That process can then no longer run firmware, which is why sweeps and replays run in fresh `spawn`ed processes (`bucky.firmware.process`).

### Reboot
`sim.reboot()` resets every peripheral and device model and re-constructs `main.cpp`'s globals in place. EEPROM survives a reboot, like real flash.

`src/firmware/fw_globals.txt` lists every firmware global. A test fails when the firmware gains one that is not handled.

## Python side (`simulation/backend/bucky/firmware`)

| Module | Purpose |
|---|---|
| `build.py`, `loader.py` | Build on demand (rebuilds only when sources change) and import `bucky_fw`. |
| `sensors.py` | Physical truth → raw readings for every sensor (table below). |
| `hardware.py` | `SimHardware`: pushes all readings into the simulated chips (`apply_truth`) and reads back motor/kicker PWM. |
| `kinematics.py` | `Motors::wheelSpeeds` geometry (M1/M2/M3 at 60°/180°/300° CW) and duty → rim speed → body twist → physics action. |
| `frames.py` | Sim frame (m, CCW, +x = attacked goal) ⇄ firmware frame (cm, x right, y forward, CW). |
| `world.py` | `FirmwareWorld`: the closed loop with the training physics (`TwoRobotPhysics`). |
| `lab.py` | Every program is a Bucky Lab module of kind `firmware`, so lab experiments, sweeps, replays and the dashboard can run the real firmware. |
| `viewer/` | pygame viewer: firmware perception, actual field, serial output. Ported from the old `tools/simulator`. |

Sensor helpers in `sensors.py`:

| Helper | Produces |
|---|---|
| `compass_raw` | LIS2MDL X/Y/Z in LSB |
| `accel_raw` | LSM303 OUT words |
| `sonar_distances_cm`, `sonar_echo_us` | Ray-cast distances and echo pulse widths |
| `ir_adc` | 16 ADC values |
| `line_adc` | `[colour][sensor]` ADC |
| `encoder_rates` | Encoder ticks/s |

## Writing a test

Unit style, one class at a time:

```python
def test_heading(sim, hw):                       # fixtures in tests/conftest.py
    c = sim.Compass(); c.begin(sim.make_sensor_bus())
    while not c.tick(): sim.sim.advance_ns(1_000_000)
    hw.set_heading(-math.radians(30))            # sim headings are CCW
    sim.sim.advance_ns(20_000_000); c.update()
    assert abs(c.getHeading() - 30) < 1.5
```

Whole program, closed loop:

```python
with FirmwareWorld("testHoldHeading", warmup_s=1.0) as w:
    w.place(robot_m=(0, 0), heading=0.0, ball_m=(0, 0.5)); w.boot(); w.run(2.0)
    print(w.serial.text, w.physics.state_a().robot_heading)
```

## Assumptions to check on the robot

The device models are faithful to the datasheets and to the firmware's own expectations, but the following mounting and calibration details are guesses, marked `UNVERIFIED` in `sensors.py` and `kinematics.py`. Each can be checked with the matching test program on the real robot.

| Assumption | Check with |
|---|---|
| Compass axes and sign | `testCompass` |
| Which sonar faces where | `testSonar` |
| IR gain, falloff and crosstalk | `testIR`, `testIRPositioning` |
| Line ring radius, LED order and levels | `testLine` |
| Encoder ticks per revolution | `testEncoder` |
| Motor deadband = `MIN_SPEED` | — |

Motor wiring follows what the repo says: M1 and M3 are reversed (`test_calibrate.cpp`, `test_hold_heading.cpp`).

## Known limits

- Interrupts only preempt at time hooks, never mid-statement, so true preemption races don't show up.
- Busy-polling loops run faster in virtual time than on the MCU; the per-call cost is a knob (`sim.set_poll_cost_ns`).
- Host libm replaces the H562's hardware CORDIC (`stm32h562xx.h` does define `CORDIC`, so the comment in `cordic.h` saying H5 has no CORDIC is out of date). Results agree to float tolerance, not bit-for-bit.
- While the line board's clock is held by hand (show mode), the ADC is not sampled.
- Only the H562RG board is modelled. The G474 needs a fake HRTIM and the EXTI encoder backend.
- Python-created firmware objects that attach interrupts (`Button`, `GPort`, `Sonar`) must not be freed while still wired: call `sim.reboot()` first. The test fixtures do this.
- `vendor/` and `boards/*/PeripheralPins.cpp` are generated by `tools/gen_vendor.py` from the PlatformIO stm32duino package. Re-run it after a framework upgrade.
