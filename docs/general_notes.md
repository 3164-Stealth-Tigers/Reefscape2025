# General Notes

## Interesting Links
- [roboRIO](https://docs.wpilib.org/en/stable/docs/software/roborio-info/index.html)
- [Programming your Radio](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)
- [Deploy Python program to roboRIO](https://docs.wpilib.org/en/stable/docs/software/python/subcommands/deploy.html)
- [Introduction to Robot Simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- [WPILib Installation Guide for 2025](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
- [Imaging the roboRio 2.0](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
- [FRC Game Tools Software](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html)

## Communications Setup

RobioRio 2.0 Controller Board Ethernet <-> [Open-Mesh OM5P-AC FIRST Robotics Competition Robot Radio](FIRST Robotics Competition Radio Open-Mesh OM5P-AC)

### 2025 update

It appears that Open-Mesh OM5P-AC may not be legal this year (https://community.firstinspires.org/2025-legal-devices) so may need to consider [VH-109 FRC Radio](https://frc-radio.vivid-hosting.net/getting-started/2025-season)
There may not be a need to include power injection ([link](https://frc-radio.vivid-hosting.net/getting-started/usage/wiring-your-radio#powering-your-radio))

# Firmware Flashing Software

Download the firmware which is part of the [FRC Game Tools Software](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html). We have no team login so you can create a personal account or download the binary from this repo.
## RoboRio 1.0

1. Connect power to RoboRio and USB cable
1. Open RoboRio Imaging Tool
1. Team Number 3164, "Format Target"* and Select the Image (in this case for 2025 it is FRC_roboRIO_2025_v2.0.zip)
1. Click on Reformat
* It is unclear at this time, but maybe during the season if there is a firmware update we may "Update Firmware" rather than "Format Target". We should verify this.

## Software

### Software running on competition laptop (Windows)

1. roboRIO Imaging Tool
1. FRC Driver Station
1. FRC Radio Configuration
1. DS4Windows
1. Gamepad Tester

## How to connect to Robot
1. Power up robot
1. Search for wifi connection with SSID: 3164
1. The FRC Driver station will show you the IP Address of the roboRio Controller. You can access the web interface directly (ex: http://10.31.64.2/#!/SystemConfig)



