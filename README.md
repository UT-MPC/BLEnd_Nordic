# BLEnd scheduler implementation for Nordic platform devices

Identifying “who is around” is key in a plethora of smart scenarios. This project provides you an implementation of the BLEnd scheduler to enable your IoT devices to be aware of its surroundings using a low duty cycle _continuous_ neighbor discovery protocol [BLEnd:practical continuous neighbor discovery for Bluetooth low energy](https://dl.acm.org/citation.cfm?id=3055086).

## Getting Started

### Prerequisites
- (Common) `ARM GCC` for compiling the SDK and your project: [Link to (GNU Arm Embedded Toolchain)](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
- (Common) nRF5x Command Line Tools `nrfjprog`: [Link to (installlation instructions from Nordic Semiconductor)](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_nrfjprogexe.html)
- (Thingy-specific) Bluetooth softdevice _s132_: [Link to (download page from Nordic Semiconductor)](https://www.nordicsemi.com/eng/nordic/Products/nRF52832/S132-SD-v4/58803)
- (Thingy-specific) Motion driver: Download the `Embedded MotionDriver` from [https://www.invensense.com/](https://www.invensense.com/) or elsewhere. Extract and put the `liblibmplmpu.a` file into `(thingy_sdk)/libs/liblibmplmpu_m4_hardfp` folder (you may need to create the folder first).
- (Thingy-specific) Install `JLinkARM` for flashing using a nRF52 PDK: [Link to (download page from SEGGER)](https://www.segger.com/jlink-software.html?step=1&file=JLink_510d).

### Installing
Clone this repository, make ```setup.sh``` executable, and run it.

Example:
```bash
$> git clone git@github.com:UT-MPC/BLEnd_Nordic.git
$> cd BLEnd_Nordic
$> chmod u+x setup.sh
$> ./setup.sh
```

## Supported Hardware and SDKs
During installation you will be asking to select one of the supported
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
...
Template project location:  (path_to_directory_on_your_disk)

 All done. You're now ready to compile the SDK and begin your BLEnd development (with the template project).
 
$> cd (path_to_directory_on_your_disk)/armgcc
$> make
$> make flash
```

## License


## Citation
Please using the following reference in your publications if it helps your research:

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
