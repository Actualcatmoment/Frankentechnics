# FrankenTechnics Turntable Controller

A microcontroller-based motor controller retrofit for a vintage Technics SL-D2 turntable — part reverse engineering, part resurrection, all DIY.

# 🛠 What is this?
This Arduino-based script replaces the original proprietary motor controller (AN630U) in a Technics SL-D2 turntable with a modern, open-source alternative using the SimpleFOC motor control 
library. It controls a 3-phase BLDC motor for precise platter rotation at 33⅓ or 45 RPM using closed-loop control with hall effect sensors, a SimpleFOC-compatible driver, and an 
Arduino-compatible microcontroller. It should thoretically work with any direct drive technics system with only slight modification.

# ⚙️ Software Features
Closed-loop FOC motor control via SimpleFOC

RPM selection based on potentiometer input (mapped to 33⅓ or 45 RPM with fine adjustment)

Strobe light synced to 60Hz visual timing marks on platter

Optional debug serial output showing raw sensor data and current speed

Configurable control constants for motion tuning

## Hardware
Microcontroller: Arduino R4 minima (or any arduino compatible MCU with atleast 85000 bytes of progmem).

Motor Driver: SimpleFOCmini or compatible BLDC driver

Motor: Original Technics 3-phase BLDC motor

Sensors: 3x Hall effect sensors for closed-loop feedback

Power Supply: Custom linear-regulated dual-voltage supply (5V + 30V)

Speed Selector: Original switch reused; analog-read mapped to RPM control

# 📦 Pinout Reference
BLDC Driver Pins
| Function | Arduino Pin |
| --- | --- | 
| Enable | 8 |
| IN3 | 9 |
| IN2	| 10 |
| IN1	| 11 |
| GND	| 12 |

Hall Sensors 
| Wire | Arduino Pin |
| --- | --- |
| H1 | A2 |
| H2 | A1 |
| H3 | A0 |

Speed Selector & Strobe
| Wire | Arduino Pin |
| --- | --- |
| Sense | A3 |
| Strobe | D2 |

# Schematic
![Franken Technics Schematic V2](https://github.com/user-attachments/assets/d90157ee-9dd6-4767-9f3a-3789fda2f242)
Just pretend that's a Minima or Uno instead of a nano. 

# 🧠 In this Repo

| File | Description |
| --- | --- |
| FT_simpleFOC.ino | closed-loop control script (to be added) |
| HallSensor.cpp & .h | Modified simpleFOC hall sensor class compatible with linear sensors (currently not working properly) |

# 🚧 Project Status
🟡 In Progress – All components are built, still fighting with posistion sensing code issues.

For whatever reason I cannot get simple FOC to detect rotation, despite the fact that it was working earlier.
Sadly I didn't backup my working version of the script so it's looking like I might be back to square one.
Maybe it's a hardware issue? Serial debug data was giving me some very weird values and the sine waves I was seeing in early tests are completely missing.

- [x] Reverse engineer and troubleshoot the old circuitboard.
- [x] Replace burnt out power transformer.
- [x] Design the power supply and sensor schematic.
- [x] Build the PSU module.
- [x] Retrofit Hall Effect Sensors.
- [x] Breakout existing motor wiring.
- [x] Breakout the existing speed selection switch and potentiometer.
  - [x] Build voltage dividing satellite board for two distinct adjustment ranges (and connect the strobe to this too for easy connection).
- [x] Connect the PSU, BLDC driver, Motor, switches, and arduino.
- [x] Make the arduino talk with the BLDC driver. (Get open-loop control working)
- [ ] Make the hall sensors talk to the arduino. (Get closed-loop control working)
  - [ ] Figure out why the hall data is so weird.
  - [ ] Find a way to implement angle sensing from three linear hall sensors set 120 degrees apart. 
   - Existing open source code expects two sensors 180 degrees apart or three digital halls.
- [ ] Implement Speed selection and adjustment.
- [ ] Tweak PID values and implement soft start.
- [ ] Tidy up the code. (also annotate it better)
- [ ] Celebrate by listening to my favorite album. (Night Gnomes by Psychedelic Porn Crumpets!)

# 📚 Credits
Thanks to:

SimpleFOC for making BLDC control more approachable

Andrew at Bushtronix for the hardware
