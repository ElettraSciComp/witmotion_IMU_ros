# Witmotion IMU sensor driver for ROS
`witmotion_imu_gps` implements a ROS 1 wrapper for [Witmotion IMU](./witmotion-uart-qt) driver library. It reads the data from the family of TTL-compatible inertial pose estimation units (IMUs) manufactured by [WitMotion Shenzhen Co.,Ltd](https://www.wit-motion.com) publishing the information in ROS-native way using [`sensor_msgs`](http://wiki.ros.org/sensor_msgs) and [`std_msgs`](http://wiki.ros.org/std_msgs) message definition packages. The module is focused on read-only access, so calibration and bias regulation functions are implemented in the underlying library. Port access model is implemented in monopolistic way acccording to UNIX specification, so only one instance of the module can be executed for the dedicated virtual device.

The initial tests of the module were done using the following Witmotion sensor devices:
- **WT31N 3-Axis Accelerometer/Gyroscope** (Linear accelerations + 2-axis Euler angles gravity tracking)
- **JY901B 9-Axis Combined IMU/Magnetometer/Altimeter** (Linear accelerations, angular velocities, Euler angles, magnetic field, barometry, altitude)

All the devices were connected to the machine using USB-TTL converter under reference schematics.

## Datasheets and official documentation
The module is developed according to the specifications released by Witmotion, the presented snapshot has download date is 23.02.2022. The official website https://wiki.wit-motion.com is not always accessible, so the PDF snapshots are placed under [IPFS web directory](https://ipfs.elettra.eu/ipfs/QmPxJCemH7JCtmr35X8Zf68pkW2GgJT6LDUrroD4fqfSnS):
- [JY901 9-Axis Combined IMU/Magnetometer/Altimeter Sensor](https://ipfs.elettra.eu/ipfs/QmPxJCemH7JCtmr35X8Zf68pkW2GgJT6LDUrroD4fqfSnS/Witmotion%20JY901%20Datasheet.pdf)
- [WT61C 3-Axis Combined Accelerometer/Gyroscope Sensor](https://ipfs.elettra.eu/ipfs/QmPxJCemH7JCtmr35X8Zf68pkW2GgJT6LDUrroD4fqfSnS/Witmotion%20WT61C%20Datasheet.pdf)
- [WT(9)31N 3-Axis Accelerometer/Gyroscope Sensor](https://ipfs.elettra.eu/ipfs/QmPxJCemH7JCtmr35X8Zf68pkW2GgJT6LDUrroD4fqfSnS/Witmotion%20WT931N%20Datasheet.pdf)

## Published message types
The module published the following message types (once the incoming data is available from the sensor, all the topic names and publisher availablility are configurable):
- [`sensor_msgs/Imu`](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html) for accelerometer/gyroscope data
- [`sensor_msgs/Temperature`](https://docs.ros.org/en/api/sensor_msgs/html/msg/Temperature.html) for thermal observation
- [`sensor_msgs/MagneticField`](https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html) for magnetometer data
- [`sensor_msgs/FluidPressure`](https://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/FluidPressure.html) for barometer output
- `std_msgs/Float64` for altimeter output

To determine which kinds of data the actually presented sensor produces, the `message-enumerator` application from an underlying library can be used.

## Configuration
Configuration of the node is done by default via the configuration YAML file [`config.yml`](./config/config.yml). But it also can be done using [`roslaunch` XML syntax](https://wiki.ros.org/roslaunch/XML) under the node's internal namespace. The single value measurements, like pressure and temperature, are enabled fot the linear calibration because there can be differences in decoding coefficients between the sensors (proven for WT31N and JY901B sensors).

### Parameters
- `port` - the virtual kernel device port name, `ttyUSB0` by default
- `baud_rate` - port rate value to be used by the library for opening the port, _9600 baud_ by default
- `polling_interval` - the sensor polling interval in milliseconds. If this parameter is omitted, the default value is set up by the library (50 ms).
- `imu_publisher:`
    - `topic_name` - the topic name for IMU data publisher, `imu` in the node's namespace by default
    - `frame_id` - IMU message header [frame ID](https://wiki.ros.org/tf)
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

