## Prerequisites
1. `ARM GCC` for compiling the SDK and your project: [Link to (GNU Arm Embedded Toolchain)](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm)
2. nRF5x Command Line Tools `nrfjprog`: [Link to (installlation instructions from Nordic Semiconductor)](http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.tools%2Fdita%2Ftools%2Fnrf5x_command_line_tools%2Fnrf5x_nrfjprogexe.html)

### (Applies to Thingy)
* Bluetooth softdevice _s132_: [Link to (download page from Nordic Semiconductor)](https://www.nordicsemi.com/eng/nordic/Products/nRF52832/S132-SD-v4/58803)
* Motion driver: Download the `Embedded MotionDriver` from [https://www.invensense.com/](https://www.invensense.com/) or elsewhere. Extract and put the `liblibmplmpu.a` file into `(thingy_sdk)/libs/liblibmplmpu_m4_hardfp` folder (you may need to create the folder first).
* Install `JLinkARM` for flashing using a nRF52 PDK: [Link to (download page from SEGGER)](https://www.segger.com/jlink-software.html?step=1&file=JLink_510d).
