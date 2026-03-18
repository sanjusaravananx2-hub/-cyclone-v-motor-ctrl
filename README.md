# Cyclone V FPGA Servo Motor Controller with LT24 LCD Display

Pure hardware servo motor controller on a DE1-SoC (Cyclone V SoC) FPGA — no CPU, no software, all parallel RTL.

## Features

- **PWM Generation** — 50Hz servo signal with 1-2ms pulse width for 0-180 degree control
- **Switch Control** — SW[0:9] maps to servo angle (0-180 degrees)
- **Auto Sweep** — KEY[1] toggles continuous sweep between 10-170 degrees
- **Preset Angles** — KEY[2] = 0 degrees, KEY[3] = 180 degrees
- **LT24 LCD Display** — Real-time angle gauge with:
  - 3-digit angle readout
  - Gradient gauge track (blue to red)
  - Animated needle with glow effect
  - Major/minor tick marks at 30-degree intervals
  - Sweep mode indicator
- **LED Diagnostics** — PWM signal mirror, clock heartbeat, sweep status

## Hardware

| Component | Details |
|-----------|---------|
| FPGA Board | Terasic DE1-SoC (5CSEMA5F31C6) |
| Servo | SG90 micro servo |
| Display | Terasic LT24 (ILI9341, 240x320, 16-bit parallel) |
| Toolchain | Quartus Prime 22.1std Lite |

## Pin Connections

### Servo (JP2 / GPIO_1)
| Wire | JP2 Pin | Signal |
|------|---------|--------|
| Yellow | Pin 1 | PWM signal |
| Orange | Pin 11 | VCC (5V) |
| Brown | Pin 12 | GND |

### LT24 Display (JP1 / GPIO_0)
Plugs directly into JP1 header, LCD screen facing toward the board.

## Controls

| Input | Function |
|-------|----------|
| SW[0:9] | Manual angle (0-180 degrees) |
| KEY[0] | Reset |
| KEY[1] | Toggle auto-sweep mode |
| KEY[2] | Preset 0 degrees (hold) |
| KEY[3] | Preset 180 degrees (hold) |

## Building

1. Open `motor_control.qpf` in Quartus Prime 22.1std
2. Run full compilation (~6 minutes)
3. Boot the DE1-SoC (wait ~30s for Linux to finish)
4. Program via JTAG:
```
quartus_pgm -m jtag -o "p;output_files/motor_control.sof@2"
```

## Project Structure

```
fpga/rtl/
  top.v           — Top-level: servo control + LT24 + HPS instantiation
  pwm_gen.v       — Configurable PWM generator
  axi_pwm_ip.v    — AXI4-Lite wrapper for PWM (for future HPS integration)
  lt24_driver.v   — ILI9341 LCD driver with gauge rendering
fpga/
  soc_system.qsys — Platform Designer system (HPS + JTAG master)
```

## Architecture

All modules run as parallel hardware on the FPGA fabric:

- **pwm_gen** generates the 50Hz PWM signal from a duty cycle input
- **Sweep FSM** produces a back-and-forth duty cycle when enabled
- **Duty mux** selects between switches, sweep, or button presets
- **lt24_driver** initializes the ILI9341 and continuously redraws the gauge
- **soc_system** provides HPS DDR3 and JTAG infrastructure
