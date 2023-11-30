** This repo is deprecated and read-only **

** Please use ess_imu_driver instead **

# README for Epson IMU Driver using SPI interface for ROS

<!---toc start-->

- [README for Epson IMU Driver using SPI interface for ROS](#readme-for-epson-imu-driver-using-spi-interface-for-ros)
  - [What is this repository for?](#what-is-this-repository-for)
  - [What kind of hardware or software will I likely need?](#what-kind-of-hardware-or-software-will-i-likely-need)
  - [How do I use the driver?](#how-do-i-use-the-driver)
  - [How do I use the driver if usleep() is not supported for time delays?](#how-do-i-use-the-driver-if-usleep-is-not-supported-for-time-delays)
  - [How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?](#how-do-i-use-the-driver-with-gpios-to-control-imu-reset-drdy-ext-pins)
  - [How do I build, install, run this ROS1 package?](#how-do-i-build-install-run-this-ros1-package)
    - [Example console output of *catkin_make* build for G366PDG0:](#example-console-output-of-catkin_make-build-for-g366pdg0)
    - [Example console output of launching ROS1 node for G365PDF1:](#example-console-output-of-launching-ros1-node-for-g365pdf1)
  - [What does this ROS1 IMU Node Publish as messages?](#what-does-this-ros1-imu-node-publish-as-messages)
    - [Without Quaternion Output](#without-quaternion-output)
      - [ROS Topic Message *data_raw*](#ros-topic-message-data_raw)
    - [With Quaternion Output](#with-quaternion-output)
      - [ROS Topic Message data](#ros-topic-message-data)
  - [Why am I seeing high latencies or slower than expected IMU data rates](#why-am-i-seeing-high-latencies-or-slower-than-expected-imu-data-rates)
  - [Package Contents](#package-contents)
  - [License](#license)
  - [References](#references)

<!---toc end-->

## What is this repository for?

- This code provides interface between Epson IMU and ROS using the SPI interface.
- This code uses the [Unofficial wiringPi](https://github.com/WiringPi/WiringPi/) library for accessing GPIO and SPI functions on the Raspberry Pi platform running Ubuntu Linux + ROS Melodic
- The src/epson_imu_spi_driver_node.cpp is the ROS C++ wrapper used to communicate with ROS
- The other source files in src/ are based on the Linux C driver originally released by Epson:
  [Epson IMU SPI-only Linux User-space Driver Example](https://vdc.epson.com/imu-products/imu-inertial-measurement-units)
- Information about ROS, ROS packages, and tutorials can be found: [ROS.org](https://www.ros.org)

## What kind of hardware or software will I likely need?

- **NOTE:** The embedded Linux host system will need the SPI interface (SCLK, MISO, MOSI) and 3 GPIOs (CS#, RST#, DRDY) enabled prior to using this software.
  - For Raspberry Pi, the SPI interface must already be enabled using raspi-config
  - This code uses a separate GPIO to manually toggle SCS# chipselect instead of the chipselect assigned to the HW SPI interface
- Epson Breakout evaluation board or some equivalent to connect to ROS host (SPI & GPIOs) [See Evaluation Boards](https://global.epson.com/products_and_drivers/sensing_system/technical_info/evaluation_tools/)
- Epson IMU (G320/G330/G354/G364/G365/G366/G370/V340) [IMU models](https://global.epson.com/products_and_drivers/sensing_system/imu/)
- ROS Melodic, Lunar, Kinetic, Indigo (via download) [ROS.org](https://www.ros.org)
- Installation guide [ROS.org](https://wiki.ros.org/ROS/Installation)
- This software was developed and tested on the following:

```
  ROS1:        Melodic
  Description: Ubuntu 18.04.4 LTS
  Release:     18.04
  Codename:    bionic
```

## How do I use the driver?

- This code assumes that the user is familiar with building ROS packages using the catkin build process.
- **NOTE:** This is *NOT* detailed instructions describing step by step procedures on how to build and install this ROS driver.
- Please refer to the ROS.org website for more detailed instructions on configuring the ROS environment & the ROS package build process. [ROS.org](https://wiki.ros.org/ROS/Tutorials)
- **NOTE:** At bare minimum, you must modify the *CMakeLists.txt*, or any time you change to a different IMU model, then build this package running "catkin_make".
- If the IMU model is unchanged, then subsequent changes to IMU settings can be done by instead editing the IMU model specific .launch file located in the launch/ folder.
- The IMU model specific launch file should only be used in conjunction with the same catkin-built executable of the same matching the IMU model.
- **NOTE:** Do not just switch .launch files without modifying the CMakeList.txt & rebuilding the executable to match the IMU model. Do not mix IMU model launch files without the matching IMU model catkin built binaries.

## How do I use the driver if usleep() is not supported for time delays?

- **NOTE:** In the hcl_rpi.c, there are wrapper functions for time delays in millisecond and microseconds using seDelayMS() and seDelayMicroSecs(), respectively.
- On embedded Linux platforms, the user may need modify and redirect to platform specific delay routines if usleep() is not supported.
- For example on RaspberryPi, the time delay functions for millisecond and microseconds can be redirected to WiringPi library delay() and delayMicroseconds(), respectively.
- If a hardware delay is not available from a library, then a software delay loop is possible but not preferred.

## How do I use the driver with GPIOs to control IMU RESET#, DRDY, EXT pins?

- Because this driver connects to the IMU using the SPI interface, the use of GPIO pins for connecting to the IMU SCS# and DRDY is mandatory (RESET# is recommended, EXT is optional).
- When possible, connecting the RESET# is recommended to force Hardware Reset during every IMU initialization, for better robustness.
- Although this code does not implement GPIO functions, this code is structured for the user to easily redirect GPIO control to low-level hardware GPIO function calls for ease of implementation.
- There is no standard method to implement GPIO connections on embedded Linux platform, but the following files typically need changes:

```
  src/hcl_rpi.c
  src/hcl_gpio_rpi.c
  src/hcl_gpio.h
```

- Typically, an external library needs to be invoked to initialize & enable GPIO HW functions on the user's embedded platform.

- This typically requires changes to hcl\_\[platform\].c, i.e. Use hcl_rpi.c as a template

  - add #include to external library near the top of hcl\_\[platform\].c
  - add the initialization call inside the seInit() function in hcl\_\[platform\].c

For example on an Raspberry Pi, the following changes can be made to hcl\_\[platform\].c:

```
...

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

...
```

- Typically, the GPIO pins need to be assigned according to pin numbering specific to the embedded HW platform.
- This typically requires changes to hcl_gpio.h

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
...

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
...
```

- Typically, the external library will have GPIO pin control functions such as set_output(), set_input(), set(), reset(), read_pin_level(), etc...

- This requires changes to hcl_gpio_rpi.c

  - Redirect function calls in hcl_gpio.c for gpioInit(), gpioRelease(), gpioSet(), gpioClr(), gpioGetPinLevel() to equivalent external library pin control functions.
  - For example on an Raspberry Pi, the following changes to hcl_gpio_rpi.c:

```
  #include "hcl.h"
  #include "hcl_gpio.h"
  #include <wiringPi.h>                         // <== Added external library

...

  int gpioInit(void)
  {
    pinMode(EPSON_RESET, OUTPUT);               // <== Added external call RESET Output Pin
    pinMode(EPSON_CS, OUTPUT);                  // <== Added external call CS# Output Pin
    pinMode(EPSON_DRDY, INPUT);                 // <== Added external call DRDY Input Pin
    pullUpDnControl(EPSON_DRDY, PUD_OFF) ;      // <== Added external call Disable any internal pullup or pulldown resistance

    return OK;
  }

...

  int gpioRelease(void)
  {
    return OK;
  }

...

  void gpioSet(uint8_t pin)
  {
    digitalWrite(pin, HIGH);                    // <== Added external call set pin HIGH
  }

  ...

  void gpioClr(uint8_t pin)
  {
    digitalWrite(pin, LOW);                     // <== Added external call set pin LOW
  }

  ...

  uint8_t gpioGetPinLevel(uint8_t pin)
  {
    return (digitalRead(pin));                  // <== Added external call to return pin state of input pin
  }

  ...
```

## How do I build, install, run this ROS1 package?

The Epson IMU ROS1 driver is designed for building in the standard ROS catkin build environment.
Therefore, a functional catkin workspace in ROS1 is a prerequisite.
Refer to the ROS1 Tutorials for more info: [ROS1 Tutorial](https://wiki.ros.org/ROS/Tutorials)

For more information on ROS & catkin setup refer to:
[Installing and Configuring ROS Environment](https://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

1. Place this package (including folders) into a new folder within your catkin workspace "src" folder.
   For example, we recommend using the folder name "ess_imu_ros1_spi_driver"

```
   <catkin_workspace>/src/ess_imu_ros1_spi_driver/
```

2. Modify the *CMakeLists.txt* to select the desired Epson IMU model that is attached to the ROS system.
   Refer to the comment lines inside the *CMakeLists.txt* for additional info.
   **NOTE:** You *MUST* re-build using *catkin_make* when changing IMU models or after any changes in the *CMakeLists.txt*

3. From the catkin workspace folder run *catkin_make* to build all ROS1 packages located in the \<catkin_workspace>/src/ folder.
   Re-run the above *catkin_make* command to rebuild the driver after making any changes to the *CMakeLists.txt*, any of the .c or .cpp or .h source files.

```
   <catkin_workspace>/catkin_make
```

**NOTE:** It is not necessary to re-run *catkin_make* if changes are only made to the launch files.

**NOTE:** It is recommended to change IMU settings by editing the parameters in the launch file, wherever possible, instead of modifying the .c or .cpp source files directly

4. Reload the current ROS environment variables that may have changed after the catkin build process.

```
   From the <catkin_workspace>: source devel/setup.bash
```

5. Modify the appropriate launch file for the IMU model in the launch/ folder to set your desired IMU configure parameter options at runtime:

| Launch Parameter | Comment                                                                                                                                                                       |
| ---------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| ext_sel          | specifies the function of the GPIO2 pin (GPIO2, External Trigger, External Counter Reset). **NOTE:** Must be set to External Counter Reset when time_correction is enabled.   |
| ext_pol          | specifies the polarity of the GPIO2 pin when *ext_sel* is External Trigger or External Counter Reset                                                                          |
| drdy_on          | specifies to enable DRDY output function on GPIO1. **NOTE:** Must be enabled when using SPI interface                                                                         |
| drdy_pol         | specifies the polarity of the DRDY output pin when enabled. **NOTE:** Must be set to 1 (active high) when using SPI interface                                                 |
| dout_rate        | specifies the IMU output data rate                                                                                                                                            |
| filter_sel       | specifies the IMU filter setting                                                                                                                                              |
| flag_out         | specifies to enable or disable ND_FLAG status in IMU output data (not used by ROS)                                                                                            |
| temp_out         | specifies to enable or disable TempC sensor in IMU output data (not used by ROS)                                                                                              |
| gyro_out         | specifies to enable or disable Gyro sensor in IMU output data (**must be enabled**)                                                                                           |
| accel_out        | specifies to enable or disable Accl sensor in IMU output data (**must be enabled**)                                                                                           |
| gyro_delta_out   | specifies to enable or disable DeltaAngle in IMU output data (not used by ROS)                                                                                                |
| accel_delta_out  | specifies to enable or disable DeltaVelocity in IMU output data (not used by ROS)                                                                                             |
| qtn_out          | specifies to enable or disable Quaternion in IMU output data (**only supported for G330/G365/G366**)                                                                          |
| atti_out         | specifies to enable or disable Attitude in IMU output data (not used by ROS)                                                                                                  |
| gpio_out         | specifies to enable or disable GPIO in IMU output data (not used by ROS)                                                                                                      |
| count_out        | specifies to enable or disable counter in IMU output data (**must be enabled when time_correction is enabled**)                                                               |
| checksum_out     | specifies to enable or disable checksum in IMU output data (when enabled checksum errors are detected)                                                                        |
| temp_bit         | specifies to 16 or 32 bit resolution in TempC output data (not used by ROS)                                                                                                   |
| gyro_bit         | specifies to 16 or 32 bit resolution in Gyro output data (**recommended to be enabled**)                                                                                      |
| accel_bit        | specifies to 16 or 32 bit resolution in Accl output data (**recommended to be enabled**)                                                                                      |
| gyro_delta_bit   | specifies to 16 or 32 bit resolution in DeltaAngle output data (not used by ROS)                                                                                              |
| accel_delta_bit  | specifies to 16 or 32 bit resolution in DeltaVelocity output data (not used by ROS)                                                                                           |
| qtn_bit          | specifies to 16 or 32 bit resolution in Quaternion output data (**only supported for G330/G365/G366**)                                                                        |
| invert_xgyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_ygyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_zgyro     | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_xaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_yaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| invert_zaccel    | specifies to reverse polarity of this sensor axis                                                                                                                             |
| atti_mode        | specifies the attitude mode as 0=inclination or 1=euler (**only supported for G330/G365/G366**)                                                                               |
| atti_profile     | specifies the attitude motion profile (**only supported for G330/G365/G366**)                                                                                                 |
| time_correction  | enables time correction function using External Counter Reset function & external 1PPS connected to IMU GPIO2/EXT pin. **NOTE:** Must have ext_sel=1 (External Counter Reset) |

**NOTE:** The ROS1 launch file passes IMU configuration settings to the IMU at runtime.
Therefore, rebuilding with *catkin_make* when changing the launch file is not required

6. To start the Epson IMU ROS1 driver use the appropriate launch file (located in launch/) from console. All parameters are described in the inline comments of the launch file. The launch file contains parameters for configuring IMU settings at runtime.

   For example, for the Epson G366PDG0 IMU:

```
   <catkin_workspace>/roslaunch ess_imu_ros1_uart_driver epson_g330_g365_g366.launch
```

| Launch File                     | Description                                                                                                                  |
| ------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| epson_g320_g354_g364.launch     | For G320PDG0/G354PDH0/G364DC0/G364PDCA, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)   |
| epson_g330_g365_g366.launch     | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data (**gyro, accel data, including quaternion orientation**)       |
| epson_g330_g365_g366_raw.launch | For G330PDG0/G365PDx1/G366PDG0, outputs to ROS topic imu/data_raw (**gyro, accel data, but no quaternion orientation**)      |
| epson_g370.launch               | For G370PDF1/G370PDS0/G370PDG0/G370PDT0 , outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**) |
| epson_v340.launch               | For V340PDD0, outputs to ROS topic imu/data_raw (**gyro, accel, but no quaternion orientation**)                             |

### Example console output of *catkin_make* build for G366PDG0:

```
guest@VDC-RPI3-BPLUS:~/catkin_ws$ catkin_make
Base path: /home/guest/catkin_ws
Source space: /home/guest/catkin_ws/src
Build space: /home/guest/catkin_ws/build
Devel space: /home/guest/catkin_ws/devel
Install space: /home/guest/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/guest/catkin_ws/build"
####
-- Using CATKIN_DEVEL_PREFIX: /home/guest/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- This workspace overlays: /home/guest/catkin_ws/devel;/opt/ros/melodic
-- Found PythonInterp: /usr/bin/python2 (found suitable version "2.7.17", minimum required is "2")
-- Using PYTHON_EXECUTABLE: /usr/bin/python2
-- Using Debian Python package layout
-- Using empy: /usr/bin/empy
-- Using CATKIN_ENABLE_TESTING: ON
-- Call enable_testing()
-- Using CATKIN_TEST_RESULTS_DIR: /home/guest/catkin_ws/build/test_results
-- Found gtest sources under '/usr/src/googletest': gtests will be built
-- Found gmock sources under '/usr/src/googletest': gmock will be built
-- Found PythonInterp: /usr/bin/python2 (found version "2.7.17")
-- Using Python nosetests: /usr/bin/nosetests-2.7
-- catkin 0.7.29
-- BUILD_SHARED_LIBS is on
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing 2 packages in topological order:
-- ~~  - ess_imu_ros1_spi_driver
-- ~~  - ess_imu_ros1_uart_driver
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ processing catkin package: 'ess_imu_ros1_spi_driver'
-- ==> add_subdirectory(ess_imu_ros1_spi_driver)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
-- Building for IMU Model: G366PDG0
---- Building for platform: RPI
---- Building for interface: SPI
-- +++ processing catkin package: 'ess_imu_ros1_uart_driver'
-- ==> add_subdirectory(ess_imu_ros1_uart_driver_gh)
-- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
---- Building for IMU Model: G370PDF1
-- Configuring done
-- Generating done
-- Build files have been written to: /home/guest/catkin_ws/build
####
#### Running command: "make -j4 -l4" in "/home/guest/catkin_ws/build"
####
Scanning dependencies of target ess_imu_ros1_spi_driver
[ 38%] Built target ess_imu_ros1_uart_driver
[ 44%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/hcl_spi_rpi.c.o
[ 50%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/hcl_rpi.c.o
[ 55%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/hcl_gpio_rpi.c.o
[ 66%] Built target ess_imu_ros1_uart_driver_node
[ 72%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/sensor_epsonCommon.c.o
[ 83%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/sensor_epsonSpi.c.o
[ 83%] Building C object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver.dir/src/sensor_epsonG330_G366.c.o
[ 88%] Linking C shared library /home/guest/catkin_ws/devel/lib/libess_imu_ros1_spi_driver.so
[ 88%] Built target ess_imu_ros1_spi_driver
Scanning dependencies of target ess_imu_ros1_spi_driver_node
[ 94%] Building CXX object ess_imu_ros1_spi_driver/CMakeFiles/ess_imu_ros1_spi_driver_node.dir/src/epson_imu_spi_driver_node.cpp.o
In file included from /usr/include/boost/bind.hpp:22:0,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/ess_imu_ros1_spi_driver/src/epson_imu_spi_driver_node.cpp:60:
/usr/include/boost/bind/bind_cc.hpp: In function ‘boost::_bi::bind_t<R, R (*)(B1), typename boost::_bi::list_av_1<A1>::type> boost::bind(R (*)(B1), A1) [with R = ros::SerializedMessage; B1 = const sensor_msgs::Imu_<std::allocator<void> >&; A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]’:
/usr/include/boost/bind/bind_cc.hpp:26:5: note: parameter passing for argument of type ‘boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >’ changed in GCC 7.1
     BOOST_BIND(BOOST_BIND_ST R (BOOST_BIND_CC *f) (B1), A1 a1)
     ^
In file included from /usr/include/boost/bind/bind.hpp:2126:0,
                 from /usr/include/boost/bind.hpp:22,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/ess_imu_ros1_spi_driver/src/epson_imu_spi_driver_node.cpp:60:
/usr/include/boost/bind/bind_cc.hpp:30:58: note: parameter passing for argument of type ‘boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >’ changed in GCC 7.1
     return _bi::bind_t<R, F, list_type> (f, list_type(a1));
                                                          ^
In file included from /usr/include/boost/bind.hpp:22:0,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/ess_imu_ros1_spi_driver/src/epson_imu_spi_driver_node.cpp:60:
/usr/include/boost/bind/bind.hpp: In constructor ‘boost::_bi::list1<A1>::list1(A1) [with A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]’:
/usr/include/boost/bind/bind.hpp:231:14: note: parameter passing for argument of type ‘boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >’ changed in GCC 7.1
     explicit list1( A1 a1 ): base_type( a1 ) {}
              ^~~~~
/usr/include/boost/bind/bind.hpp:231:44: note: parameter passing for argument of type ‘boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >’ changed in GCC 7.1
     explicit list1( A1 a1 ): base_type( a1 ) {}
                                            ^
In file included from /usr/include/boost/bind/bind.hpp:47:0,
                 from /usr/include/boost/bind.hpp:22,
                 from /opt/ros/melodic/include/ros/publisher.h:35,
                 from /opt/ros/melodic/include/ros/node_handle.h:32,
                 from /opt/ros/melodic/include/ros/ros.h:45,
                 from /home/guest/catkin_ws/src/ess_imu_ros1_spi_driver/src/epson_imu_spi_driver_node.cpp:60:
/usr/include/boost/bind/storage.hpp: In constructor ‘boost::_bi::storage1<A1>::storage1(A1) [with A1 = boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >]’:
/usr/include/boost/bind/storage.hpp:42:14: note: parameter passing for argument of type ‘boost::reference_wrapper<const sensor_msgs::Imu_<std::allocator<void> > >’ changed in GCC 7.1
     explicit storage1( A1 a1 ): a1_( a1 ) {}
              ^~~~~~~~
[100%] Linking CXX executable /home/guest/catkin_ws/devel/lib/ess_imu_ros1_spi_driver/ess_imu_ros1_spi_driver_node
[100%] Built target ess_imu_ros1_spi_driver_node

```

### Example console output of launching ROS1 node for G366PDG0:

```
guest@VDC-RPI3-BPLUS:~/catkin_ws$ roslaunch ess_imu_ros1_spi_driver epson_g330_g365_g366.launch
... logging to /home/guest/.ros/log/d724c9b0-f4e1-11ed-8836-b827eba23167/roslaunch-VDC-RPI3-BPLUS-6984.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started roslaunch server http://VDC-RPI3-BPLUS:41607/

SUMMARY
========

PARAMETERS
 * /ess_imu_ros1_spi_driver_node/accel_bit: 1
 * /ess_imu_ros1_spi_driver_node/accel_delta_bit: 1
 * /ess_imu_ros1_spi_driver_node/accel_delta_out: 0
 * /ess_imu_ros1_spi_driver_node/accel_out: 1
 * /ess_imu_ros1_spi_driver_node/atti_bit: 1
 * /ess_imu_ros1_spi_driver_node/atti_conv: 0
 * /ess_imu_ros1_spi_driver_node/atti_mode: 1
 * /ess_imu_ros1_spi_driver_node/atti_out: 0
 * /ess_imu_ros1_spi_driver_node/atti_profile: 0
 * /ess_imu_ros1_spi_driver_node/checksum_out: 1
 * /ess_imu_ros1_spi_driver_node/count_out: 1
 * /ess_imu_ros1_spi_driver_node/dout_rate: 4
 * /ess_imu_ros1_spi_driver_node/drdy_on: 1
 * /ess_imu_ros1_spi_driver_node/drdy_pol: 1
 * /ess_imu_ros1_spi_driver_node/ext_pol: 0
 * /ess_imu_ros1_spi_driver_node/ext_sel: 1
 * /ess_imu_ros1_spi_driver_node/filter_sel: 5
 * /ess_imu_ros1_spi_driver_node/flag_out: 1
 * /ess_imu_ros1_spi_driver_node/gpio_out: 0
 * /ess_imu_ros1_spi_driver_node/gyro_bit: 1
 * /ess_imu_ros1_spi_driver_node/gyro_delta_bit: 1
 * /ess_imu_ros1_spi_driver_node/gyro_delta_out: 0
 * /ess_imu_ros1_spi_driver_node/gyro_out: 1
 * /ess_imu_ros1_spi_driver_node/invert_xaccel: 0
 * /ess_imu_ros1_spi_driver_node/invert_xgyro: 0
 * /ess_imu_ros1_spi_driver_node/invert_yaccel: 0
 * /ess_imu_ros1_spi_driver_node/invert_ygyro: 0
 * /ess_imu_ros1_spi_driver_node/invert_zaccel: 0
 * /ess_imu_ros1_spi_driver_node/invert_zgyro: 0
 * /ess_imu_ros1_spi_driver_node/qtn_bit: 1
 * /ess_imu_ros1_spi_driver_node/qtn_out: 1
 * /ess_imu_ros1_spi_driver_node/temp_bit: 1
 * /ess_imu_ros1_spi_driver_node/temp_out: 1
 * /ess_imu_ros1_spi_driver_node/time_correction: 0
 * /rosdistro: melodic
 * /rosversion: 1.14.13

NODES
  /
    ess_imu_ros1_spi_driver_node (ess_imu_ros1_spi_driver/ess_imu_ros1_spi_driver_node)

auto-starting new master
process[master]: started with pid [6998]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to d724c9b0-f4e1-11ed-8836-b827eba23167
process[rosout-1]: started with pid [7011]
started core service [/rosout]
process[ess_imu_ros1_spi_driver_node-2]: started with pid [7014]
[ INFO] [1684348639.306385789]: Initializing HCL layer...

Initializing libraries......done.[ INFO] [1684348639.318485861]: Initializing GPIO interface...
[ INFO] [1684348639.318933933]: Initializing SPI interface...

...sensorDummyWrite.[ INFO] [1684348639.520160522]: Checking sensor NOT_READY status...
...done.[ INFO] [1684348640.344544274]: Initializing Sensor...
[ INFO] [1684348640.350763059]: Epson IMU initialized.
[ INFO] [1684348640.351130350]: Compiled for:	G366PDG0
[ INFO] [1684348640.351315766]: Reading device info...
[ INFO] [1684348640.352995033]: PRODUCT ID:	G366PDG0
[ INFO] [1684348640.354876017]: SERIAL ID:	00000008

...Sensor start.[ INFO] [1684348640.362137195]: Quaternion Output: Native.

```

## What does this ROS1 IMU Node Publish as messages?

The Epson IMU ROS1 driver will publish IMU messages as convention per [REP 145](http://www.ros.org/reps/rep-0145.html).

- For IMU models such as G320/G354/G364/G370/V340, the IMU messages will only update the fields for angular rate (gyro) and linear acceleration (accel) data.
- For IMU models G330/G365/G366, it depends on the enabling/disabling of the internal attitude function with quaternion output:
  - IMU messages will only update *angular_velocity* (gyro) and *linear_acceleration* (accel) fields when quaternion output is *disabled*
  - IMU messages will update update *angular_velocity* (gyro), *linear_acceleration* (accel), and *orientation* field using the internal extended Kalman Filter when the quaternion output is *enabled*

### Without Quaternion Output

For non-quaternion output models, the ROS1 driver will publish to the following ROS topic:

```
/epson_imu/data_raw <-- orientation field will not contain valid data & should be ignored
```

**NOTE** The launch file will remap the ROS message to publish on /imu/data_raw

#### ROS Topic Message *data_raw*

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

- For quaternion output models, the ROS1 driver will publish to the following ROS topic:

```
/epson_imu/data <-- orientation, angular_velocity, linear_acceleration fields will be updating
```

**NOTE** The launch file will remap the ROS message to publish on /imu/data

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
   src/           <== source code for ROS node C++ wrapper, IMU Linux C driver, and additional README_src.md specifically for building and using the IMU Linux C driver as stand-alone (without ROS support)
   CHANGELOG.rst  <== summarizes major changes
   CMakeLists.txt <== cmake build script for catkin_make
   LICENSE.txt    <== description of the applicable licenses
   package.xml    <== catkin package description
   README.md      <== general README for the ROS1 driver
```

## License

Refer to *LICENSE.txt* in this package

## References

1. https://index.ros.org
