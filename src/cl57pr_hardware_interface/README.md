# cl57pr_hardware_interface

A `ros2_control` `SystemInterface` plugin for a single-axis **STEPPERONLINE CL57PR**
closed-loop stepper driver, talking Modbus RTU over RS485 via **libmodbus**.

## What it does

- Exposes one joint with a `position` command interface and `position` /
  `velocity` state interfaces.
- On activation: connects over RS485, clears any latched alarm, enables the
  motor, and loads the motion profile (start speed, accel/decel time, cruise
  speed) once — only the target position is rewritten every control cycle.
- Each `write()`: converts the commanded joint angle (rad) to a signed 32-bit
  pulse count and sends it as an **absolute, interruptible** positioning move
  (bits `POS_START | POS_ABSOLUTE | MODE_SWITCH` in the motion command
  register), so a new setpoint can override an in-flight move.
- Each `read()`: pulls error code, motion status, and position/speed in a
  single contiguous 7-register Modbus read (`0x0006`–`0x000C`) for efficiency,
  and converts pulses/rpm to radians / rad-s⁻¹.
- On deactivation: issues a normal (decelerated) stop and releases the motor.

## Register map

The register addresses in `cl57pr_hardware_interface.hpp` match the CL57PR /
DM-PR series map (model at `0x0000`, motion status at `0x0007`, motion command
at `0x0027`, etc.) — this is the same map used by the vendor Python example
this package was built from. **Note:** this differs from the register map
used in some CL57R Arduino/RS485 code (e.g. `ENABLE_MOTOR` at `0x0038` there
vs. an auxiliary-command word at `0x002D` here). If your particular unit
doesn't behave as expected, read back `DRIVER_MODEL` (`0x0000`) and confirm
against your driver's manual before trusting the rest of the map — some
CL57 variants ship with different firmware/register layouts under similar
model names.

## Build

```bash
sudo apt install libmodbus-dev
colcon build --packages-select cl57pr_hardware_interface
```

## Hardware parameters

| Parameter | Default | Meaning |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | RS485 adapter device |
| `baud_rate` | `9600` | Must match the driver's SW6/SW7 DIP switches |
| `slave_id` | `1` | Modbus node address, matches SW1–SW5 |
| `steps_per_rev` | `10000` | Pulses per motor revolution (microstep or encoder-based) — sets the rad ↔ pulses conversion |
| `max_speed_rpm` | `120` | Cruise speed for positioning moves |
| `start_speed_rpm` | `30` | Starting speed |
| `accel_time_ms` / `decel_time_ms` | `200` / `200` | Ramp times |
| `zero_position_on_activate` | `true` | Zero the driver's pulse counter on activation (that physical pose becomes joint 0) |
| `position_epsilon_rad` | `0.0005` | Deadband to avoid re-sending unchanged setpoints |

See `config/example_ros2_control.xacro` for a working `<ros2_control>` block.

## Things you'll likely want to adapt

- **Homing.** This interface assumes the axis already has a known, repeatable
  physical reference at startup (either `zero_position_on_activate` zeroes it
  there, or you disable that and trust the driver's persisted counter). A
  proper homing routine (trigger `HOME_START`, poll `HOMING_DONE`) isn't
  wired in — add it in `on_activate()` if your mechanism needs it.
- **Alarms.** `write()` currently just stops sending new commands while
  `HAS_ALARM` is set and logs at 1 Hz; it doesn't auto-clear. Wire a
  service or a `command_interface` bit if you want the controller to be able
  to trigger `CLEAR_ALARM` at runtime.
- **Multiple axes.** This is deliberately single-joint/single-slave. For
  several CL57PR drivers on one shared RS485 bus, the cleanest extension is
  one `modbus_t*` per slave ID reused across N joints in one `SystemInterface`
  (rather than N processes fighting over the same serial port).
- **Velocity control.** Only position mode is wired up. The driver also
  supports a speed-mode command bit (`SPEED_START`) if you need velocity
  control instead.

## Files

```
cl57pr_hardware_interface/
├── CMakeLists.txt
├── package.xml
├── cl57pr_hardware_interface.xml        # pluginlib description
├── include/cl57pr_hardware_interface/
│   └── cl57pr_hardware_interface.hpp
├── src/
│   └── cl57pr_hardware_interface.cpp
└── config/
    └── example_ros2_control.xacro
```
