# BLEnd scheduler implementation for Nordic platform devices
[![License](https://img.shields.io/badge/license-BSD-blue.svg)](LICENSE)

Identifying “who is around” is key in a plethora of smart scenarios. This project provides you an implementation of the BLEnd scheduler to enable your IoT devices to be aware of its surroundings using a low duty cycle _continuous_ neighbor discovery protocol [BLEnd:practical continuous neighbor discovery for Bluetooth low energy](https://dl.acm.org/citation.cfm?id=3055086).

As a communication middleware, this implementation of the blend protocol provides an energy-efficient approach to let your device automatically discovers its peer devices, using **ad hoc** wireless connections, without infrastructure support(e.g. Access Point).

Applications can also piggyback information in the discovery beacons. For instance, the [Stacon system](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=8730667&tag=1) leverages this symmetric and asynchronous peer discovery to enable the ad hoc grouping of sensing devices and collaboration.

## Getting Started

### Prerequisites
- (Common) `ARM GCC` for compiling the SDK and your project: [Link to (GNU Arm Embedded Toolchain)](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
- (Common) nRF5x Command Line Tools `nrfjprog`: [Link to download page](https://www.nordicsemi.com/Software-and-Tools/Development-Tools/nRF5-Command-Line-Tools), [Link to install instructions](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_nrfjprogexe.html)
- (Thingy-specific) Bluetooth softdevice 4.0.5 S132: [Link to (download page from Nordic Semiconductor)](https://www.nordicsemi.com/Software-and-Tools/Software/S132/Download)
- (Thingy-specific) Motion driver: Create an account at [https://www.invensense.com/](https://www.invensense.com/). Under "Developers -> Software Downloads" download the `Embedded MotionDriver 6.12`. Under the path `motion_driver_6.12/mpl libraries/arm/gcc4.9.3`, unzip `liblibmplmpu_m4_hardfp.zip` and put the `liblibmplmpu.a` file into `(thingy_sdk)/libs/liblibmplmpu_m4_hardfp` folder (you may need to create the folder first).
- (Thingy-specific) Install `JLinkARM` for flashing using a nRF52 PDK: [Link to (download page from SEGGER)](https://www.segger.com/jlink-software.html?step=1&file=JLink_510d).

### Installing
Clone this repository, make ```setup.sh``` executable, and run it.

Example:
```bash
$> git clone git@github.com:UT-MPC/BLEnd_Nordic.git
$> cd BLEnd_Nordic
$> chmod u+x setup.sh
$> ./setup.sh
($> run SDK setup script in the downloaded directory)
```

### Specify blend parameters
See ```blend_param_t```

### Percom Demo (Stacon):
> Run the setup script with option **2** and then check out `src/blend_project_templates/Thingy_IoT_SensorKit_v2.1.0/percom_demo`.

## Supported Hardware and SDKs
During installation you will be asked to select one of the supported
version before and the script will automatically download and
instrument the SDK you need.

| Device | Description | SDK Version |
| --- | --- | --- |
| [nRF52840 DK](https://www.nordicsemi.com/eng/Products/nRF52840-DK) | Development kit for nRF52840 from Nordic Semiconductors  | nRF5 v14 |
| [nRF52840 DK](https://www.nordicsemi.com/eng/Products/nRF52840-DK) | (Same as above)  | nRF5 v15 |
| [Thingy52](https://www.nordicsemi.com/eng/Products/Nordic-Thingy-52) | IoT sensor kit from Nordic Semiconductors  | v2.1.0 |

## Usage
Please pay attention to the output of the ```setup``` script. You might need to compile the SDK manually (_urls_ to the guide will be provided). It will also prompt the location of an example project.
Just go into the directory and try compiling/flashing the application.

Example:
```bash
(script outputs)
Template project location:  (path_to_directory_on_your_disk)
 All done. You're now ready to compile the SDK and begin your development (with the template project).

$> cd (path_to_directory_on_your_disk)/armgcc
$> make flash
```

## Debug terminal (RTT)
1. Connect to the DEV board using `JLinkExe` (or connect Thingy52 via a SWD cable):
   - Start JLinkExe in the terminal.
   ```bash
     JLinkExe  -device nrf52 -if swd -speed 4000;
   ```
   - Run `connect` in JLinkExe:
   ```bash
   JLink>connect
   ```
   - (And you should see a message)
   ```bash
   Cortex-M4 identified.
   ```
   
2. Open the RTT client:
```bash
     JLinkRTTClient
```

## License and Citation
The BLEnd scheduler and the sample projects are released under the BSD 3-Clause license.

Please use the following reference in your publications if this project helps your research:

_BLEnd Scheduler_
```
@inproceedings{julien2017blend,
  title={BLEnd: practical continuous neighbor discovery for Bluetooth low energy},
  author={Julien, Christine and Liu, Chenguang and Murphy, Amy L and Picco, Gian Pietro},
  booktitle={Proceedings of the 16th ACM/IEEE International Conference on Information Processing in Sensor Networks},
  pages={105--116},
  year={2017},
  organization={ACM}
}
```
