# E-puck2 Robotics Project

_A course project for MICRO-315 as part of MT-BA6 at EPFL, May 2022._

Designed to run on the e-puck2 mini robot, featuring an ARM Cortex-M4 CPU
processor on the STM32F4 microcontroller. Built using of ChibiOS, a free
realtime operating system and their custom build of the Eclipse IDE. Programmed
in C with some additional Python scripts.

## Description

This is a course project written in C with the purpose of teaching us about
embedded systems, interfacing with hardware and functioning within the
constraints of a RTOS.

<!--
This program displays a welcome message, prompts the user to select a temperature unit, allows the user to calibrate the server motor and then displays the measured temperature on the screen, also moving the servo to a specified position, proportional to the measured value. By pushing down on the analogue encoder, a synchronous interrupt is triggered, allowing the selection of a new unit and servo recalibration.
-->

## Prerequisites

This project requires the following additional dependencies:

- [ChibiOS](https://www.chibios.org/dokuwiki/doku.php)
- [e-puck2/e-puck2_main-processor](https://github.com/e-puck2/e-puck2_main-processor)
- [cvra/msgbus](https://github.com/cvra/msgbus)
- [ARM gcc compiler](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain)
  (optional)
- [epuck-2 Eclipse IDE w/ gcc compiler](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development)
  (optional)

Download these dependencies and place them in respective folders _adjacent_ to
this repository in the following structure:

```text
.
├── lib/
│   ├── ChibiOS/
│   │   └── ...
│   ├── e-puck2_main-processor/
│   │   └── ...
│   └── msgbus/
│       └── ....
├── micro-315-project/
│   ├── micro-315-project/
│   │   ├── main.c
│   │   ├── Makefile
│   │   └── ...
│   ├── README.md (this README)
│   └── ...
└── toolchain/
    ├── arm-none-eabi/
    │   ├── include/
    │   │   └── ...
    │   └── ...
    └── bin/
        ├── arm-none-eabi-gcc.exe
        └── ...
```

It's also possible to create a symlink to the Eclipse IDE's toolchain:
`toolchain` ->
`/full/path/to/Eclipse_e-puck2/Tools/gcc-arm-none-eabi-7-2017-q4-major-win32`.

**Important**: due to some limitations, the Eclipse IDE must be placed somewhere
where the path does **not** contain any spaces!

## Building

The project can be built with the included Makefile. This will produce a
`micro-315-project.elf` executable under `build`, which can be flashed onto the
e-puck2.

### With the Eclipse IDE

Download and install the e-puck2 distribution of the Eclipse IDE
([wiki](https://www.gctronic.com/doc/index.php?title=e-puck2_robot_side_development)).
This will also require Java version 8 (not higher) to run correctly. Open the
project within Eclipse and build the project. It already contains the
`arm-none-eabi` toolchain and is preconfigured, therefore the `toolchain` folder
in the diagram above is not required. It does help, however, if you wish to use
VSCode and benefit from Intellisense.

Running `Build Project` from Eclipse will launch the Makefile task. This will
produce the executable which can then be easily loaded onto the device using the
`micro-315-project` debug configuration. Before launching the configuration,
find out which COM port the e-puck2 device is connected to for the GDB server
(via USB), and configure this in the debug configuration under `Variables`.

> On Windows, you can use Device Manager to inspect COM ports, find the one with
> a `Bus reported description` of `e-puck2 (GDB)`, for me it was `COM8`.

Launching the debug configuration will load the executable onto the device and
start a debugging session, allowing you to step through code and inspect CPU
registers on the device itself using the `EmbSys Registers` panel. It will
immediately pause on execution, click the play button to resume.

Once the debug configuration has been run once, the robot will retain the
program within its flash memory. It can be disconnected from the USB cable and
run again without having to re-flash the program.

## Usage

_TODO_

## Authors

- Marcus Cemes
- Alexandre Dodens

## License

This project is released under the MIT license.

See [LICENSE](LICENSE).
