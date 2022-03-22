# AUVSystem

## Setup

After cloning, mark scripts as executables:

```bash
chmod +x *.sh
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
- SIMULATION - switches on some things. For now, it switches updating state of the AUV based on Model, not from proper feedback

## Run

Simply:

```bash
./runAll.sh
```

If you want GUI for tests, launch it separately in another terminal. (Will be changed possibly in future to be like **Build**)

```bash
python3 haller_gui.py
```