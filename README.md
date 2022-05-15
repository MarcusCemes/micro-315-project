# E-puck2 robotics project

_A course project for MICRO-315 as part of MT-BA6 at EPFL, May 2022._

Designed to run on the e-puck2 mini robot, featuring an ARM Cortex-M4 CPU
processor and an STM32F4 MCU. Built using ChibiOS, the e-puck2_main-processor
library, written in C and compiled using the the GCC ARM toolchain with some
additional scripts in Python.

### Features

- 🤖 Realtime hardware digital signal processing (FFT)
- 👂Phase difference acoustic localisation (DTOA)
- 🚗 Constant-acceleration movement profile
- 📱 Realtime Bluetooth visualisation
- 🧵 RTOS multithreading
- 🚨 Cool lights
- 🔊 Cool sounds

## Description

This is an EPFL Bachelor's course project written in C with the purpose of
teaching us about embedded systems, interfacing with hardware and functioning
within the constraints of a RTOS.

Our robot attempts to find the exact location of sound source in the room using
a _Difference in Time of Arrival_ technique. The ARM Cortex-M4 features a
_Digital Signal Processor_ (DSP) which is able to perform signal analysis in
realtime. The DSP is capable of computing a _Fast Fourier Transform_, whose
magnitude and complex argument is vital for this approach. The FFT allows us to
isolate each frequency component, not only giving us the phase of the same
sinusoidal wave as it arrives at each individual microphone, but also allowing
us to easily reject any other audio frequencies in the room, such as music
playing next to the robot.

The back microphone is used to isolate the frequency component with the highest
magnitude (using a tone generator on a phone, for example), which is then used
for relative phase-difference comparison with the three other microphones to
calculate δ. The e-puck2 is able to find the exact angle up to a theoretical 5.6
kHz, after which more than one mode will appear (it will continue to work when
aligned _sufficiently_ well with the source).

In theory, it should be possible to isolate a sound source in 3D space using the
four microphone array, but we found that in practice the non-ideal positioning
of the microphones made this difficult. Instead, we use the L/R microphones to
get a rough estimate of the angle and the F/B microphones to isolate the
quadrant in which the sound source is in. Our audio processing is done in-place
on existing buffers for memory efficiency, using only 24 KiB of RAM for
processing 1024 samples from each microphone (one FFT complex float buffer, four
PCM float buffers).

<div align="center">
  <br />
  <img alt="A diagram of the DTOA technique" src="./assets/images/dtoa-diagram.svg">
  <br />
  <em>Figure: A diagram illustrating the Difference of Time of Arrival technique</em>
</div>
<br />
<div align="center">
  <br />
  <img alt="DTOA equations" src="./assets/images/dtoa-equations.svg" height="40em">
  <br />
  <em>Equation: Main DTOA equations</em>
</div>
<br />

Through an iterative approach, the robot is able to converge on the direction of
the sound source with remarkable precision. With at least two sampled points and
some trigonometry, the distance to the source can be determined. A custom
implementation of the `motors.c` library allowed us to implement motor control
that is precise down to a single step (one wheel revolution is 1000 steps). To
further increase movement precision, a constant-acceleration profile is applied
to movement during acceleration and deceleration to minimise wheel sleep due to
the discrete nature of the step motors.

Additional features include splitting tasks into different priority threads,
realtime proximity detection, custom light animations, speaker sound effects and
realtime Bluetooth communication using a custom serial protocol based on
MessagePack.

---

## Building

The build process can be a bit cumbersome, there's a few ways about it.

### Using the CI release

This repository has an associated GitHub Action workflow that builds the project
into a `micro-315-project.elf` executable. You can grab the build artefact, or
use the workflow file as a reproduction of a working build setup on Linux.

### Using the GCC toolchain

This is the cleanest approach to compiling the project, but can be a bit
difficult to set up, depending on your familiarity with toolchains and using the
terminal.

You will need:

- A copy of this repository
- A copy of the
  [e-puck2_main-processor](https://github.com/e-puck2/e-puck2_main-processor)
  library (with all submodules!)
- The
  [gcc-arm-none-eabi-9-2019-q4](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/9-2019-q4-major)
  ARM toolchain

Here's a quick script to fetch the two repositories:

```bash
$ git clone https://github.com/MarcusCemes/micro-315-project
$ mkdir lib
$ cd lib
$ git clone --recursive https://github.com/e-puck2/e-puck2_main-processor
```

> **Note**: you may also need to clone
> [ChibiOS](https://github.com/ChibiOS/ChibiOS) and
> [cvra/msgbus](https://github.com/cvra/msgbus) repositories in the `lib`
> directory, adjacent to `e-puck2_main-processor`, but this _shouldn't_ be
> necessary, as they are already submodules of the `e-puck2_main-processor` and
> get picked up by the Makefile.

You should have the following directory structure:

```text
.
├── lib/
│   └── e-puck2_main-processor/
│       └── ...
└── micro-315-project/
    ├── micro-315-project/
    │   ├── main.c
    │   ├── Makefile
    │   └── ...
    ├── README.md  (this README)
    └── ...

gcc-arm-none-eabi-9-2019-q4/
├── arm-none-eabi/
│   ├── include/
│   │   └── ...
│   └── ...
└── bin/  (add to PATH)
    ├── arm-none-eabi-gcc.exe
    └── ...
```

The `gcc-arm-none-eabi-9-2019-q4` toolchain can be placed anywhere on your
system, just don't forget to add the `bin` folder to your PATH (it won't work
without it!).

You should now be able to navigate to the inner `micro-315-project` folder and
run the Makefile:

```bash
$ cd micro-315-project/micro-315-project
$ make all
```

> **Note**: if you are on Windows, you will need some additional GNU tools on
> your PATH for this to work, such as `make` and some other common utilities.
> This is less trivial, but can be achieved by installing
> [MSYS2](https://www.msys2.org) and adding the `bin` folder to your PATH (find
> the one containing `bash.exe`). If it still doesn't work, try running
> `pacman -S base-devel` from the MSYS2 prompt.

The executable can be flashed to the robot either with a dedicated tool (yet to
find one!), or via GDB (that's what we used) using either the Cortex-Debug
VSCode extension (see below) or the Eclipse IDE (see below).

### With the Eclipse IDE

This is the recommended approach to compiling the project, but can be a bit
cumbersome.

The complete installation steps can be found on the
[e-puck2 website](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development).

You will need:

- Java 8 32-bit (not higher!)
- The Eclipse_e-puck2 package

> **Note**: The Eclipse IDE **must** be placed somewhere where there are **no**
> spaces in the absolute path. If your username has a space, your desktop will
> not do!

The Eclipse IDE also provides an easy approach to flashing the compiled
executable onto the robot, using the included `micro-315-project` debug
configuration.

> **Note**: You will have to find out which COM port (on Windows) or file
> (/dev/cu... on macOS/Linux) the GDB server of the robot is advertised on and
> configure this in the debug configuration. See
> [this](https://www.gctronic.com/doc/index.php?title=e-puck2#Finding_the_USB_serial_ports_used)
> more more information.

Launching the debug configuration will load the executable onto the device and
start a debugging session, allowing you to step through code and inspect CPU
registers on the device itself using the `EmbSys Registers` panel. It will
immediately pause on execution, click the play button to resume.

Once the debug configuration has been run once, the robot will retain the
program within its flash memory. It can be disconnected from the USB cable and
run again without having to re-flash the program.

## Usage

Once the executable has been flashed, the robot can be turned on to immediately
boot into the program. There are two programs available:

1. Pointing towards a sound source
2. Locating a sound source

The programs can be chosen at boot using the rotating selector wheel, between
positions 0 (9 o'clock) and 1 (one step clockwise from 0).

The first mode will simply rotate the robot and align itself with a sound
source, whilst the second will sample three points to determine the direction
and distance to the source, playing a little celebratory song afterwards.
Booping the robot gently at any time will set off the infrared proximity sensors
and result in a warning light sequence.

It's best to use a tone generator on a phone around 2 kHz, pointing towards the
robot. Make sure you are in an acoustically suitable environment, reflections
off of walls can throw it off.

That's pretty much it. There's a lot going on behind the scenes, take a look at
the source code to see what's going on.

### Sounds

The robot will additionally play sounds from a microSD card slot. The necessary
WAV files are located under `assets/sounds`, just place them in the root of a
microSD card and insert it into the e-puck2.

The robot will work without a microSD card as well, just with less music.

### Realtime visualisation

This repository includes a Python script under `receiever` that connects to the
robot over Bluetooth for realtime visualisation of captured PCM data as well as
the resulting FFT. To view any other data, use the `comms_send_buffer()`
function from `comms.h` to send over any arbitrary data that can be processed by
the Python and Numpy script (see the existing examples for PCM data).

## VSCode integration

In order to circumvent the Eclipse IDE, we've managed configure a suitable
replacement using the following VSCode extensions:

- C/C++ (for Intellisense)
- Makefile tools (analyse the build script, configure headers, definitions, ...)
- Cortex-Debug (a fantastic replacement for Eclipse's Debug Configurations)
- Downloading the ARM toolchain and adding to path (see above)

We've found VSCode to be faster and more reliable. You will find the relevant
configuration in the `.vscode` folder, including a build task, debug
configuration, etc. Configuring VSCode will require a similar build setup to the
GCC toolchain setup described above.

## References

- [e-puck2 homepage](https://www.gctronic.com/e-puck2.php)
- [Course homepage](https://isa.epfl.ch/imoniteur_ISAP/!itffichecours.htm?ww_i_matiere=2705910539&ww_x_anneeAcad=2021-2022&ww_i_section=945244&ww_i_niveau=6683117&ww_c_langue=en)
- [ARM embedded toolchain](https://developer.arm.com/downloads/-/gnu-rm)
- [DSP CMSIS library](https://arm-software.github.io/CMSIS_5/General/html/index.html)
- [Acoustic location](https://en.wikipedia.org/wiki/Acoustic_location)
  (Wikipedia)

## Authors

- Marcus Cemes
- Alexandre Dodens

## License

This project is released under the MIT license.

See [LICENSE](LICENSE).
