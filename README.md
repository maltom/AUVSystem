# AUVSystem

## Setup

Clone this repo to your favourite directory.
Make sure you have installed:

- git
- gcc and g++ 9.4 or newer
- CMake 3.16 or newer
- Python 3.8.10 or newer
- ROS Noetic
- boost library
- Eigen3 library
- Control Toolbox library
- LAPACK
- OpenCV 4.0 (not yet)
- CUDA 11 (not yet)

After cloning, mark scripts as executables:

```bash
cd AUVSystem
chmod +x *.sh
```

It is highly possible that the newest version of software is on develop branch. Switch to that:

```bash
git switch develop
```

### Important

To use any script, you have to be in the same directory in your terminal as the scripts you want to run.

```bash
# DON'T DO THAT - WON'T WORK
AUVSystem/buildAll.sh
# OR THAT
../AUVSystem/buildAll.sh
# OR THAT
../buildAll.sh
```

## Build

Use buildAll.sh in terminal with settings of your choice as arguments.

Default command builds in "production" mode and in Release CMake setting:

```bash
./buildAll.sh
```

To add options, simply write them in CAPS after ```./buildAll.sh```. Example:

```bash
./buildAll.sh NOLQR DEBUG
```

The above version changes build type to Debug (allowing launching the Debugger with one of System nodes) and defines NOLQR macro which switches down LQR regulator in ThrustRegulator.

Available options:

- RELEASE - setting build type to Release (default)
- DEBUG - setting build type to Debug
- NOLQR - switching off LQR in **ThrusterRegulator** Node. Instead of taking new target position from ROS Topics, it takes forces to set from ArbitrarlySetThrustForces Topic which is published by haller_gui.
- SIMULATION - switches on some things. For now, it switches updating state of the AUV based on Model, not from proper feedback and publishing estimated global position which is calculated by LQR and model
- MANUAL - switches off thrusters regulator messages to thrusters and servos
- NOSTM - switches down hardware checks for STM. Use when the STM32 is not connected
- NODVL - the same as above but for DVL
- NOZED - the same as above but for ZED
- NOHYDROPHONES - the same as above but for hydrophones
- NOHARDWARE - includes all: NOSTM, NODVL, NOZED, NOHYDROPHONE

## Run

First, run ```roscore``` in any terminal to launch ROS master.

Then, simply run in another terminal:

```bash
./runAll.sh
```

If you want GUI for tests, launch it separately in another terminal. (Will be changed possibly in future to be like **Build**)

```bash
python3 haller_gui.py
```

## Kill

To kill all processes from **AUVSystem** just run in any terminal command:

```bash
./killAll.sh
```

This essentially kills the whole system, leaving ```roscore``` intact.

## Good Luck, commander
