# [Witmotion IMU sensor driver for ROS](https://wiki.ros.org/witmotion_ros)
[![ROS1 CI](https://github.com/ElettraSciComp/witmotion_IMU_ros/actions/workflows/main.yml/badge.svg)](https://github.com/ElettraSciComp/witmotion_IMU_ros/actions/workflows/main.yml)
|`focal-source`|`focal-amd64`|`focal-armhf`|`focal-arm64`|`dev`|`doc`|
|--------------|-------------|-------------|-------------|-----|-----|
|[![focal-source](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__witmotion_ros__ubuntu_focal__source)](https://build.ros.org/view/Nsrc_uF/job/Nsrc_uF__witmotion_ros__ubuntu_focal__source)|[![focal-amd64](http://build.ros.org/buildStatus/icon?job=Nbin_uF64__witmotion_ros__ubuntu_focal_amd64__binary)](https://build.ros.org/view/Nbin_uF64/job/Nbin_uF64__witmotion_ros__ubuntu_focal_amd64__binary)|[![focal-armhf](http://build.ros.org/buildStatus/icon?job=Nbin_ufhf_uFhf__witmotion_ros__ubuntu_focal_armhf__binary)](https://build.ros.org/view/Nbin_ufhf_uFhf/job/Nbin_ufhf_uFhf__witmotion_ros__ubuntu_focal_armhf__binary)|[![focal-arm64](http://build.ros.org/buildStatus/icon?job=Nbin_ufv8_uFv8__witmotion_ros__ubuntu_focal_arm64__binary)](https://build.ros.org/view/Nbin_ufv8_uFv8/job/Nbin_ufv8_uFv8__witmotion_ros__ubuntu_focal_arm64__binary)|[![dev](http://build.ros.org/buildStatus/icon?job=Ndev__witmotion_ros-release__ubuntu_focal_amd64)](https://build.ros.org/view/Ndev/job/Ndev__witmotion_ros-release__ubuntu_focal_amd64)|[![doc](http://build.ros.org/buildStatus/icon?job=Ndoc__witmotion_ros-release__ubuntu_focal_amd64)](https://build.ros.org/view/Ndoc/job/Ndoc__witmotion_ros-release__ubuntu_focal_amd64)|

`witmotion_ros` module implements a ROS 1 wrapper for [Witmotion IMU](https://github.com/ElettraSciComp/witmotion_IMU_QT) driver library. It reads the data from the family of TTL-compatible inertial pose estimation units (IMUs) manufactured by [WitMotion Shenzhen Co.,Ltd](https://www.wit-motion.com) publishing the information in ROS-native way using [`sensor_msgs`](http://wiki.ros.org/sensor_msgs) and [`std_msgs`](http://wiki.ros.org/std_msgs) message definition packages. The module is focused on read-only access, so calibration and bias regulation functions are implemented in the underlying library. Port access model is implemented in monopolistic way acccording to UNIX specification, so only one instance of the module can be executed for the dedicated virtual device.

## Datasheets and official documentation
The module is developed according to the specifications released by Witmotion, the presented snapshot has download date is 23.02.2022. The official website https://wiki.wit-motion.com is not always accessible, so the PDF snapshots are placed under [IPFS web directory](https://bafybeie4dvr3mjax4f2qbf25etizzfkofpepffvpx5nejcx6bu6g53kcji.ipfs.twdragon.net/).

## ROS2 branch
The ROS2 compatible implementation of the driver is WIP under [`ros2`](https://github.com/ElettraSciComp/witmotion_IMU_ros/tree/ros2) branch. The initial migration made by [`@fllay`](https://github.com/fllay) (pull request [#12](https://github.com/ElettraSciComp/witmotion_IMU_ros/pull/12)).

### Marking ROS2-related issues and pull requests
The contributors are strongly encouraged to mark the names of their ROS2-related issues and pull requests with `[ROS2]` prefix.

## Installation

### Prerequisites
The package requires QtSerialPort development package from Qt 5.2+
```sh
sudo apt-get install libqt5serialport5-dev
```

### Building
```sh
cd catkin_ws
git clone --recursive https://github.com/ElettraSciComp/witmotion_IMU_ros.git src/witmotion_ros
catkin_make
```
If compilation fails, first check the directory `src/witmotion_ros/witmotion-uart-qt`. If it is empty, the recursive clone failed, and you should manually clone the underlying library from the repository https://github.com/ElettraSciComp/witmotion_IMU_QT into this directory. **IMPORTANT!** Please beware of the directory name, the `CMakeLists` file refers exactly to the name `witmotion-uart-qt` specified in the target import section.


## Usage
```sh
roslaunch witmotion_ros witmotion.launch
```

## Configuration
Configuration of the node is done by default via the configuration YAML file [`config.yml`](./config/config.yml). But it also can be done using [`roslaunch` XML syntax](https://wiki.ros.org/roslaunch/XML) under the node's internal namespace. The single value measurements, like pressure and temperature, are enabled for the linear calibration because there can be differences in decoding coefficients between the sensors (proven for WT31N and JY901B sensors).

### Parameters
- `port` - the virtual kernel device name for a port, `ttyUSB0` by default
- `baud_rate` - port rate value to be used by the library for opening the port, _9600 baud_ by default
- `polling_interval` - the sensor polling interval in milliseconds. If this parameter is omitted, the default value is set up by the library (50 ms).
- `timeout_ms` - the sensor timeout period in milliseconds.  If no data is received from the sensor after this period, then an error is raised and the node terminates. If this parameter is omitted, a default value of 3 times the polling interval is used.  If this parameter is zero, the timeout check is disabled.
- `restart_service_name` - the service name used to restart the sensor connection after an error.
- `imu_publisher:`
    - `topic_name` - the topic name for IMU data publisher, `imu` in the node's namespace by default
    - `frame_id` - IMU message header [frame ID](https://wiki.ros.org/tf)
    - `use_native_orientation` - instructs the node to use the native quaternion orientation measurement from the sensor instead of synthesized from Euler angles. **NOTE**: if this setting is enabled bu the sensor does not produce orientation in the quaternion format, the IMU message will never be published!
    - `measurements` - every measurement in IMU message data pack can be enabled or disabled. If the measurement is disabled, the corresponding covariance matrix is set to begin from `-1` as it is described in the [message definition](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html).
        - `acceleration`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
        - `angular_velocity`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
        - `orientation`
            - `enabled`
            - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
- `temperature_publisher`
    - `enabled` - enable or disable temperature measurement extraction
    - `topic_name` - the topic name for publishing temperature data
    - `frame_id` - message header frame ID
    - `from_message` - the message type string to determine from which type of Witmotion measurement message the temperature data should be extracted (please refer to the original documentation for detailed description). The possible values are: `acceleration`, `angular_vel`, `orientation` or `magnetometer`.
    - `variance` - the constant variance, if applicable, otherwise 0
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
- `magnetometer_publisher`
    - `enabled` - enable or disable magnetometer measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `frame_id` - message header frame ID
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
    - `covariance` - row-major matrix 3x3, all zeros for unknown covariation
- `barometer_publisher`
    - `enabled` - enable or disable barometer measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `frame_id` - message header frame ID
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
    - `variance` - the constant variance, if applicable, otherwise 0
- `altimeter_publisher`
    - `enabled` - enable or disable altitude measurement extraction
    - `topic_name` - the topic name for publishing the data
    - `coefficient` - linear calibration multiplier, 1.0 by default
    - `addition` - linear calibration addendum, 0 by default
- `orientation_publisher`
    - `enabled` - enable or disable orientation measurement extraction
    - `topic_name` - the topic name for publishing the data
- `gps_publisher`
    - `enabled` - enables/disables all GPS receiver measurements extraction
    - `navsat_fix_frame_id` - frame ID for GPS fixed position publisher
    - `navsat_fix_topic_name` - topic name for GPS fixed position publisher
    - `navsat_altitude_topic_name` - topic name for GPS altitude publisher
    - `navsat_satellites_topic_name` - topic name for GPS active satellites number publisher
    - `navsat_variance_topic_name` - topic name for GPS diagonal variance publisher
    - `ground_speed_topic_name` - topic name for GPS ground speed publisher
- `rtc_publisher`
    - `enabled` - enables/disables realtime clock information decoder
    - `topic_name` - topic name for realtime clock publisher
    - `presync` - instructs the node to perform an attempt to pre-synchronize sensor's internal realtime clock
