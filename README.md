# README for Epson IMU Driver using SPI interface for ROS1 Node

## What is this repository for?

* This code provides interface between Epson IMU (newer G325/G365/G370 or mature G320/G354/G364/V340) and ROS1 using the SPI interface.
* This code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux + ROS Melodic
* The src/epson_imu_spi_driver_node.cpp is the ROS1 C++ wrapper used to communicate with ROS1
* The other source files in src/ are based on the C driver released by Epson:
  [Epson IMU SPI-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
* Information about ROS1, and tutorials can be found: [ROS.org](https://wiki.ros.org/)


## What kind of hardware or software will I likely need?

* *NOTE:* The embedded Linux host system will need the SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY) enabled prior to using this software.
  - For Raspberry Pi, the SPI interface must already be enabled using raspi-config
  - This code uses a separate GPIO to manually toggle SCS# chipselect instead of the chipselect assigned to the HW SPI interface
* Epson Breakout evaluation board or some equivalent to connect to ROS host (SPI & GPIOs) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
* Epson IMU (V340/G320/G325/G354/G364/G365/G370) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
* ROS Melodic, Lunar, Kinetic, Indigo (via download) [ROS.org](https://www.ros.org)
* Installation guide [ROS.org](https://wiki.ros.org/ROS/Installation)
* This software was developed and tested on the following:
```
  ROS1:        Melodic
  Description: Ubuntu 18.04.4 LTS
  Release:     18.04
  Codename:    bionic
```

## How do I use the driver?

* This code assumes that the user is familiar with building ROS1 packages using the catkin build process.
* This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS1 driver.
* Please refer to the ROS.org website for more detailed instructions on configuring the ROS environment & the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)
* *NOTE:* At bare minimum, you must modify the CMakeLists.txt to select the desired IMU model, and build the package atleast once or any time you change to a different IMU model.
* If the IMU model is *unchanged*, then subsequent changes to IMU settings can be done by editing the IMU model specific launch file in the launch folder.
* The IMU model specific launch file should only be used with the same catkin-built executable matching the IMU model.
* *NOTE* Do not mix IMU model launch files & IMU model catkin built binaries.


## How do I use the driver if usleep() is not supported for time delays?

* NOTE: In the hcl_rpi.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
* On embedded Linux platforms, these may need to be redirected to HW platform specific delay routines.
* For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
* If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

* Because this driver connects to the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU SCS# and DRDY is mandatory (RESET# is recommended, EXT is optional).
* When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization for better robustness.
* This code is structured to easily redirect to low-level hardware GPIO function calls for easy implementation such as RaspberryPi.
* There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_rpi.c
  src/hcl_gpio.c
  src/hcl_gpio.h
```

* Typically, an external library needs to be invoked to initialize & enable GPIO HW functions.

* This typically requires changes to hcl_[platform].c, i.e. Use hcl_rpi.c as a template

  - add #include to external library near the top of hcl_[platform].c
  - add the initialization call inside the seInit() function in hcl_[platform].c

For example on an Raspberry Pi, the following changes can be made to hcl_[platform].c:

```
  .
  .
  .
  #include <stdint.h>
  #include <stdio.h>
  #include <wiringPi.h>  // <== Added external library

  int seInit(void)
  {
    // Initialize wiringPi libraries                                                   // <== Added
    printf("\r\nInitializing libraries...");                                           // <== Added
    if(wiringPiSetupGpio() != 0) {                                                     // <== Added external library initialization
      printf("\r\nError: could not initialize wiringPI libraries. Exiting...\r\n");    // <== Added
      return NG;                                                                       // <== Added
    }                                                                                  // <== Added
    printf("...done.");

    return OK;
  }
  .
  .
  .

```

* Typically, the GPIO pins need to be assigned according to pin numbering specific to the HW platform.

* This typically requires changes to hcl_gpio.h

For example on an Raspberry Pi, the following changes to hcl_gpio.h with the following pin mapping:

```
    Epson IMU                   Raspberry Pi
    ---------------------------------------------------
    EPSON_RESET                 RPI_GPIO_P1_15 (GPIO22) Output
    EPSON_DRDY                  RPI_GPIO_P1_18 (GPIO24) Input
    EPSON_CS                    RPI_GPIO_P1_16 (GPIO23) Output

```
Note: The RPI SPI0_cs0 is not connected. Chip select is being manually controlled via GPIO on P1_16. 

```
  // Prototypes for generic GPIO functions
  int gpioInit(void);
  int gpioRelease(void);

  void gpioSet(uint8_t pin);
  void gpioClr(uint8_t pin);
  uint8_t gpioGetPinLevel(uint8_t pin);

  #define RPI_GPIO_P1_15              22                    // <== Added
  #define RPI_GPIO_P1_16              23                    // <== Added
  #define RPI_GPIO_P1_18              24                    // <== Added

  #define EPSON_RESET                 RPI_GPIO_P1_15        // <== Added
  #define EPSON_CS                    RPI_GPIO_P1_16        // <== Added 
  #define EPSON_DRDY                  RPI_GPIO_P1_18        // <== Added
  .
  .
  .
```


* Typically, the external library will have GPIO pin control functions such as set_output, set_input, set, reset, read_pin_level, etc...

* This requires changes to hcl_gpio.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.

  - For example on an Raspberry Pi, the following changes to hcl_gpio.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library
  .
  .
  .

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call CS# Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }
  .
  .
  .
  int gpioRelease(void)
  {
    return OK;
  }
  .
  .
  .
  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  .
  .
  .
  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  .
  .
  .
  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  .
  .
  .
```


## How do I build, install, run this ROS1 package?

The Epson IMU ROS1 driver is designed to build in the ROS catkin build environment.
Therefore, a functional catkin workspace in ROS1 is a prerequisite.
Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

For more information on ROS & catkin setup refer to
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).


1. Place this package (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "epson_imu_spi_ros"
```
   <catkin_workspace>/src/epson_imu_spi_ros/ <-- place files here
```
2. Modify the CMakeLists.txt to select the desired Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the CMakeLists.txt for additional info.
   *NOTE:* You *MUST* re-build using catkin build when changing IMU models or any changes in the CmakeLists.txt

3. From the catkin workspace folder run "catkin_make" to build all ROS1 packages located in the <catkin_workspace>/src/ folder.

```
   <catkin_workspace>/catkin_make
```
   Re-run the above "catkin_make" command to rebuild the driver after making any changes to the CMakeLists.txt or any of the .c or .cpp or .h source files.
   It is not necessary to "catkin_make" if changes are only made to the launch files

   *NOTE:* It is recommended to change IMU settings by editing the parameters in the launch file, wherever possible, instead of modifying the .c or .cpp source files directly

4. Reload the current ROS environment variables that may have changed after the catkin build process.
```
   From the <catkin_workspace>: devel/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:


Parameter            | Comment
-------------------- | -------------
ext_sel              | specifies the function of the GPIO2 (GPIO2, External Trigger, External Counter Reset)
ext_pol              | specifies the polarity of the GPIO2 pin when External Trigger or External Counter Reset is selected
drdy_on              | specifies to enable DRDY function on GPIO1
drdy_pol             | specifies the polarity of the DRDY input pin when enabled
dout_rate            | specifies the IMU output data rate
filter_sel           | specifies the IMU filter setting
flag_out             | specifies to enable or disable ND_FLAG status in IMU output data (not used by ROS)
temp_out             | specifies to enable or disable TempC sensor in IMU output data (not used by ROS)
gyro_out             | specifies to enable or disable Gyro sensor in IMU output data (must be enabled)
accel_out            | specifies to enable or disable Accl sensor in IMU output data (must be enabled)
gyro_delta_out       | specifies to enable or disable DeltaAngle in IMU output data (not used by ROS)
accel_delta_out      | specifies to enable or disable DeltaVelocity in IMU output data (not used by ROS)
qtn_out              | specifies to enable or disable Quaternion in IMU output data (only support for G325/G365)
atti_out             | specifies to enable or disable Attitude in IMU output data (not used by ROS)
gpio_out             | specifies to enable or disable GPIO in IMU output data (not used by ROS)
count_out            | specifies to enable or disable counter in IMU output data (must be enabled when time_correction is enabled)
checksum_out         | specifies to enable or disable checksum in IMU output data (when enabled checksum errors are detected)
temp_bit             | specifies to 16 or 32 bit resolution in TempC output data (not used by ROS)
gyro_bit             | specifies to 16 or 32 bit resolution in Gyro output data
accel_bit            | specifies to 16 or 32 bit resolution in Accl output data
gyro_delta_bit       | specifies to 16 or 32 bit resolution in DeltaAngle output data (not used by ROS)
accel_delta_bit      | specifies to 16 or 32 bit resolution in DeltaVelocity output data (not used by ROS)
qtn_bit              | specifies to 16 or 32 bit resolution in Quaternion output data (only support for G325/G365)
invert_xgyro         | specifies to reverse polarity of this sensor axis
invert_ygyro         | specifies to reverse polarity of this sensor axis
invert_zgyro         | specifies to reverse polarity of this sensor axis
invert_xaccel        | specifies to reverse polarity of this sensor axis
invert_yaccel        | specifies to reverse polarity of this sensor axis
invert_zaccel        | specifies to reverse polarity of this sensor axis
atti_mode            | specifies the attitude mode as 0=inclination or 1=euler (only support for G325/G365)
atti_profile         | specifies the attitude motion profile (supported only for G325PDF1 or G365PDF1)
time_correction      | enables time correction function using IMU counter reset function & external 1PPS connection to IMU GPIO2/EXT pin. Must have ext_sel=1 (external counter reset)

   *NOTE:* The ROS1 launch file passes IMU configuration settings to the IMU at runtime.
           Therefore does not need rebuilding with catkin when changing the launch file.

6. To start the Epson IMU ROS1 driver use the appropriate launch file (located in launch/) from console.

   For example, for the Epson G365 IMU:
```
   <catkin_workspace>/roslaunch epson_imu_spi_driver epson_g325_g365.launch
```

   - The launch file contains parameters for configuring settings at runtime:
   - All parameters are described in the inline comments of the launch file.

     *epson_g320_g354_g364.launch*
       - For G320/G354/G364, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)

     *epson_g325_g365.launch*
       - For G325PDF1/G365PDx1, outputs to ROS topic imu/data (gyro, accel data, including quaternion orientation)

     *epson_g325_g365_raw.launch*
       - For G325PDF0/G365PDx0, outputs to ROS topic imu/data_raw (gyro, accel data, but no quaternion orientation)

     *epson_g370.launch*
       - For G370, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)

     *epson_v340.launch*
       - For V340, outputs to ROS topic imu/data_raw (gyro, accel, but no quaternion orientation)


### Example console output of catkin build for G365PDF1:
```
guest@guest-desktop:~/catkin_ws$ catkin_make
Base path: /home/guest/catkin_ws
Source space: /home/guest/catkin_ws/src
Build space: /home/guest/catkin_ws/build
Devel space: /home/guest/catkin_ws/devel
Install space: /home/guest/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/guest/catkin_ws/build"
####
####
#### Running command: "make -j4 -l4" in "/home/guest/catkin_ws/build"
####
Scanning dependencies of target epson_imu_spi_driver_lib
make[2]: Warning: File '/home/guest/catkin_ws/src/imu_ros_spi_wip_rpi/src/sensor_epsonG365.c' has modification time 29 s in the future
[ 11%] Building C object imu_ros_spi_wip_rpi/CMakeFiles/epson_imu_spi_driver_lib.dir/src/sensor_epsonG365.c.o
[ 22%] Linking C shared library /home/guest/catkin_ws/devel/lib/libepson_imu_spi_driver_lib.so
[ 77%] Built target epson_imu_spi_driver_lib
[ 88%] Linking CXX executable /home/guest/catkin_ws/devel/lib/epson_imu_spi_driver/epson_imu_spi_driver_node
[100%] Built target epson_imu_spi_driver_node

```


### Example console output of launching ROS1 node for G365PDF1:
```
guest@guest-desktop:~/catkin_ws$ roslaunch epson_imu_spi_driver epson_g325_g365.launch
... logging to /home/guest/.ros/log/8963c3d4-0411-11eb-8ac1-b827eba23167/roslaunch-guest-desktop-18362.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://guest-desktop:42841/

SUMMARY
========

PARAMETERS
 * /epson_imu_spi_driver_node/accel_bit: 1
 * /epson_imu_spi_driver_node/accel_delta_bit: 1
 * /epson_imu_spi_driver_node/accel_delta_out: 0
 * /epson_imu_spi_driver_node/accel_out: 1
 * /epson_imu_spi_driver_node/atti_bit: 1
 * /epson_imu_spi_driver_node/atti_conv: 0
 * /epson_imu_spi_driver_node/atti_mode: 1
 * /epson_imu_spi_driver_node/atti_out: 0
 * /epson_imu_spi_driver_node/atti_profile: 1
 * /epson_imu_spi_driver_node/checksum_out: 1
 * /epson_imu_spi_driver_node/count_out: 1
 * /epson_imu_spi_driver_node/dout_rate: 4
 * /epson_imu_spi_driver_node/drdy_on: 1
 * /epson_imu_spi_driver_node/drdy_pol: 1
 * /epson_imu_spi_driver_node/ext_pol: 0
 * /epson_imu_spi_driver_node/ext_sel: 1
 * /epson_imu_spi_driver_node/filter_sel: 5
 * /epson_imu_spi_driver_node/flag_out: 1
 * /epson_imu_spi_driver_node/gpio_out: 0
 * /epson_imu_spi_driver_node/gyro_bit: 1
 * /epson_imu_spi_driver_node/gyro_delta_bit: 1
 * /epson_imu_spi_driver_node/gyro_delta_out: 0
 * /epson_imu_spi_driver_node/gyro_out: 1
 * /epson_imu_spi_driver_node/invert_xaccel: 0
 * /epson_imu_spi_driver_node/invert_xgyro: 0
 * /epson_imu_spi_driver_node/invert_yaccel: 0
 * /epson_imu_spi_driver_node/invert_ygyro: 0
 * /epson_imu_spi_driver_node/invert_zaccel: 0
 * /epson_imu_spi_driver_node/invert_zgyro: 0
 * /epson_imu_spi_driver_node/qtn_bit: 1
 * /epson_imu_spi_driver_node/qtn_out: 1
 * /epson_imu_spi_driver_node/temp_bit: 1
 * /epson_imu_spi_driver_node/temp_out: 1
 * /epson_imu_spi_driver_node/time_correction: 0
 * /rosdistro: melodic
 * /rosversion: 1.14.9

NODES
  /
    epson_imu_spi_driver_node (epson_imu_spi_driver/epson_imu_spi_driver_node)

auto-starting new master
process[master]: started with pid [18372]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 8963c3d4-0411-11eb-8ac1-b827eba23167
process[rosout-1]: started with pid [18383]
started core service [/rosout]
process[epson_imu_spi_driver_node-2]: started with pid [18390]
[ INFO] [1601575901.739119874]: Initializing HCL layer...

Initializing libraries......done.[ INFO] [1601575901.746968937]: Initializing GPIO interface...
[ INFO] [1601575901.747402852]: Initializing SPI interface...

...sensorDummyWrite.[ INFO] [1601575901.948265031]: Checking sensor NOT_READY status...
...done.[ INFO] [1601575902.772941465]: Initializing Sensor...
[ INFO] [1601575902.784431173]: Epson IMU initialized.
[ INFO] [1601575902.786159176]: PRODUCT ID:     G365PDF1
[ INFO] [1601575902.788027755]: SERIAL ID:      X0000013

...Sensor start.[ INFO] [1601575902.793125513]: Quaternion Output: Native.
 
```


## What does this ROS1 IMU Node Publish as an Messages?

The Epson IMU ROS1 driver will publish messages which will vary slightly on the IMU model and output configuration.
- For IMU models such as G320/G354/G364/G370/V340, the IMU messages will only contain fields for angular rate (gyro) and linear acceleration (accel) data.
- For IMU models G325/G365 there is internal attitude function with quaternion output:
  - IMU messages can output orientation using the internal extended Kalman Filter when the quaternion output is *enabled*.
  - IMU messages will only contain angular rate (gyro) and linear acceleration (accel) data when quaternion output is *disabled*.

The Epson IMU ROS1 driver will publish IMU messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

### Without Quaternion Output
For non-quaternion output models, the ROS1 driver will publish to the following ROS topics:
```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```
*NOTE* The launch file will remap the message to publish on /imu/data_raw


#### ROS Topic Message data_raw
```
---
header:
  seq: 34983
  stamp:
    secs: 1601590579
    nsecs: 771273325
  frame_id: "imu_link"
orientation:
  x: 2.4786295434e+33
  y: 1.1713334935e+38
  z: 1.17130631507e+38
  w: 1.17130956026e+38
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 0.00435450254008
  y: 0.000734272529371
  z: -9.40820464166e-05
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.730921983719
  y: -1.54766368866
  z: 9.72711181641
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---

```

### With Quaternion Output
For quaternion output models, the ROS1 driver will publish to the following ROS topics:

```
/epson_imu/data <-- orientation field will contain quaternions
```
*NOTE* The launch file will remap the message to publish on /imu/data

#### ROS Topic Message data

```
---
header:
  seq: 10608
  stamp:
    secs: 1601575987
    nsecs: 673387357
  frame_id: "imu_link"
orientation:
  x: -0.0801454782486
  y: 0.0367396138608
  z: 0.00587898213416
  w: 0.996088504791
orientation_covariance: [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: -0.00118702522013
  y: 0.000320095219649
  z: -0.00014466587163
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.727666378021
  y: -1.5646469593
  z: 9.69056034088
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
---
```


## Why am I seeing high latencies or slower than expected IMU data rates
   
This will largely depend on your host system processing load and latency.
If your ROS platform is running too many ROS node packages or simply too
slow it may not be able detect the rising edge of the IMU DRDY signal and
then burst read the IMU sampling data and post process it.

Try modifying the *dout_rate* and *filter_sel* to the slowest setting that
can meet your ROS system requirements. 

Monitoring the DRDY signal on the IMU to verify the stability of the IMU
DRDY signal is recommended when experiencing data rate issues.


## Package Contents

The Epson IMU ROS1 driver-related sub-folders & root files are:
```
   launch/        <== various example launch files for Epson IMU models
   src/           <== source code for ROS1 node C++ wrapper, IMU C driver, and additional README_src.md
   visual         <== example python2 3D visualizer using the deprecated python-visual library
   CmakeLists.txt <== build script for catkin build
   package.xml    <== catkin package description
   README.md      <== general README for the ROS1 driver
```

## License

### The Epson IMU C++ Wrapper ROS1 Node is released under BSD-3 license.

[This software is BSD-3 licensed.](http://opensource.org/licenses/BSD-3-Clause)

Original Code Development:
Copyright (c) 2019, Carnegie Mellon University. All rights reserved.

Additional Code contributed:
Copyright (c) 2020, Seiko Epson Corp. All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

### The Epson IMU C driver software is released as public domain.

THE SOFTWARE IS RELEASED INTO THE PUBLIC DOMAIN.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
NONINFRINGEMENT, SECURITY, SATISFACTORY QUALITY, AND FITNESS FOR A
PARTICULAR PURPOSE. IN NO EVENT SHALL EPSON BE LIABLE FOR ANY LOSS, DAMAGE
OR CLAIM, ARISING FROM OR IN CONNECTION WITH THE SOFTWARE OR THE USE OF THE
SOFTWARE.


## References
1. https://index.ros.org
