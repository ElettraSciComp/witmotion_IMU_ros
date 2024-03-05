#include "witmotion_ros.h"

bool ROSWitmotionSensorController::suspended = false;
rclcpp::Service<std_srvs::srv::Empty>::SharedPtr restart_service;

/* IMU */

rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
std::string ROSWitmotionSensorController::imu_frame_id = "imu";
bool ROSWitmotionSensorController::imu_enable_accel = false;
bool ROSWitmotionSensorController::imu_enable_velocities = false;
bool ROSWitmotionSensorController::imu_enable_orientation = false;
bool ROSWitmotionSensorController::imu_have_accel = false;
bool ROSWitmotionSensorController::imu_have_velocities = false;
bool ROSWitmotionSensorController::imu_have_orientation = false;
bool ROSWitmotionSensorController::imu_native_orientation = false;
std::vector<double> ROSWitmotionSensorController::imu_accel_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};
std::vector<double> ROSWitmotionSensorController::imu_velocity_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};
std::vector<double> ROSWitmotionSensorController::imu_orientation_covariance = {
    -1, 0, 0, 0, 0, 0, 0, 0, 0};

/* TEMPERATURE */
witmotion_packet_id ROSWitmotionSensorController::temp_from = pidAcceleration;
std::string ROSWitmotionSensorController::temp_frame_id = "imu";
bool ROSWitmotionSensorController::temp_enable = false;
float ROSWitmotionSensorController::temp_variance = 0.f;
float ROSWitmotionSensorController::temp_coeff = 1.f;
float ROSWitmotionSensorController::temp_addition = 0.f;
rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher;

/* MAGNETOMETER */
std::string ROSWitmotionSensorController::magnetometer_frame_id = "imu";
bool ROSWitmotionSensorController::magnetometer_enable = false;
std::vector<double> ROSWitmotionSensorController::magnetometer_covariance = {
    0, 0, 0, 0, 0, 0, 0, 0, 0};
rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr
    magnetometer_publisher;
float ROSWitmotionSensorController::magnetometer_coeff = 1.f;
float ROSWitmotionSensorController::magnetometer_addition = 0.f;

/* BAROMETER */
std::string ROSWitmotionSensorController::barometer_frame_id = "imu";
bool ROSWitmotionSensorController::barometer_enable = false;
double ROSWitmotionSensorController::barometer_variance = 0.f;
double ROSWitmotionSensorController::barometer_coeff = 1.f;
double ROSWitmotionSensorController::barometer_addition = 0.f;
rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr
    barometer_publisher;

/* ALTIMETER */
bool ROSWitmotionSensorController::altimeter_enable = false;
double ROSWitmotionSensorController::altimeter_coeff = 1.f;
double ROSWitmotionSensorController::altimeter_addition = 0.f;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr altimeter_publisher;
bool ROSWitmotionSensorController::have_altitude = false;
double ROSWitmotionSensorController::last_altitude = 0.f;

/* ORIENTATION */
bool ROSWitmotionSensorController::orientation_enable = false;
rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr
    orientation_publisher;

/* GPS */
bool ROSWitmotionSensorController::gps_enable = false;
bool ROSWitmotionSensorController::have_gps = false;
bool ROSWitmotionSensorController::have_accuracy = false;
bool ROSWitmotionSensorController::have_ground_speed = false;
uint32_t ROSWitmotionSensorController::satellites = 0;
float ROSWitmotionSensorController::gps_altitude = NAN;
std::vector<double> ROSWitmotionSensorController::gps_covariance = {
    0, 0, 0, 0, 0, 0, 0, 0, 0};
std::string ROSWitmotionSensorController::gps_frame_id = "world";


rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr ground_speed_publisher;
rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gps_altitude_publisher;
rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr accuracy_publisher;
rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr satellites_publisher;

/* REALTIME CLOCK */
bool ROSWitmotionSensorController::rtc_enable = false;
rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr rtc_publisher;
bool ROSWitmotionSensorController::rtc_presync = false;

ROSWitmotionSensorController::ROSWitmotionSensorController()
    : reader_thread(dynamic_cast<QObject *>(this))
{

  // In case we need string to float conversions this prevents locale dependant conversions
  std::locale::global(std::locale::classic());

  /*Initializing ROS fields*/
  node = rclcpp::Node::make_shared("witmotion");


  /*Initializing ROS fields*/
  node->declare_parameter("restart_service_name", "restart");
  _restart_service_name = node->get_parameter("restart_service_name")
                              .get_parameter_value()
                              .get<std::string>();

  restart_service = node->create_service<std_srvs::srv::Empty>(
      _restart_service_name, &ROSWitmotionSensorController::Restart);

  /* IMU */
  node->declare_parameter("imu_publisher.topic_name", "imu");
  _imu_topic = node->get_parameter("imu_publisher.topic_name")
                   .get_parameter_value()
                   .get<std::string>();

  imu_publisher = node->create_publisher<sensor_msgs::msg::Imu>(_imu_topic, 10);

  node->declare_parameter("imu_publisher.frame_id", "imu");
  imu_frame_id = node->get_parameter("imu_publisher.frame_id")
                     .get_parameter_value()
                     .get<std::string>();

  node->declare_parameter("imu_publisher.use_native_orientation", false);
  imu_native_orientation =
      node->get_parameter("imu_publisher.use_native_orientation")
          .get_parameter_value()
          .get<bool>();

  node->declare_parameter("imu_publisher.measurements.acceleration.enabled", false);
  imu_enable_accel =
      node->get_parameter("imu_publisher.measurements.acceleration.enabled")
          .get_parameter_value()
          .get<bool>();

  load_parameter(imu_enable_accel, "imu_publisher.measurements.acceleration.covariance", -1.0, imu_accel_covariance);

  node->declare_parameter("imu_publisher.measurements.angular_velocity.enabled",
                          false);
  imu_enable_velocities =
      node->get_parameter("imu_publisher.measurements.angular_velocity.enabled")
          .get_parameter_value()
          .get<bool>();

  load_parameter(imu_enable_velocities,"imu_publisher.measurements.angular_velocity.covariance", -1.0, imu_velocity_covariance);

  node->declare_parameter("imu_publisher.measurements.orientation.enabled",
                          false);
  imu_enable_orientation =
      node->get_parameter("imu_publisher.measurements.orientation.enabled")
          .get_parameter_value()
          .get<bool>();

  load_parameter(imu_enable_orientation,"imu_publisher.measurements.orientation.covariance", -1.0, imu_orientation_covariance);

  /* TEMPERATURE */
  node->declare_parameter("temperature_publisher.enabled", false);
  temp_enable = node->get_parameter("temperature_publisher.enabled")
                    .get_parameter_value()
                    .get<bool>();

  if (temp_enable) {
    node->declare_parameter("temperature_publisher.topic_name", "temperature");
    _temp_topic = node->get_parameter("temperature_publisher.topic_name")
                      .get_parameter_value()
                      .get<std::string>();

    node->declare_parameter("temperature_publisher.frame_id", "temp_frame");
    temp_frame_id = node->get_parameter("temperature_publisher.frame_id")
                        .get_parameter_value()
                        .get<std::string>();

    std::string temp_from_str;
    node->declare_parameter("temperature_publisher.from_message",
                            "from_message");
    temp_from_str = node->get_parameter("temperature_publisher.from_message")
                        .get_parameter_value()
                        .get<std::string>();

    std::transform(temp_from_str.begin(), temp_from_str.end(),
                   temp_from_str.begin(), ::toupper);
    if (temp_from_str == "ACCELERATION")
      temp_from = pidAcceleration;
    else if (temp_from_str == "ANGULAR_VEL")
      temp_from = pidAngularVelocity;
    else if (temp_from_str == "ORIENTATION")
      temp_from = pidAngles;
    else if (temp_from_str == "MAGNETOMETER")
      temp_from = pidMagnetometer;
    else {
      RCLCPP_WARN_SKIPFIRST(rclcpp::get_logger("ROSWitmotionSensorController"),
                            "Cannot determine message type to take temperature "
                            "from (%s). Falling back to acceleration message",
                            temp_from_str.c_str());
      temp_from = pidAcceleration;
    }

    load_parameter_f("temperature_publisher.variance",  0.f, temp_variance);
    load_parameter_f("temperature_publisher.coefficient",  1.f, temp_coeff);
    load_parameter_f("temperature_publisher.addition",  0.f, temp_addition);
    
    temp_publisher =
        node->create_publisher<sensor_msgs::msg::Temperature>(_temp_topic, 1);
  }
  /* MAGNETOMETER */
  node->declare_parameter("magnetometer_publisher.enabled", false);
  magnetometer_enable = node->get_parameter("magnetometer_publisher.enabled")
                            .get_parameter_value()
                            .get<bool>();
  if (magnetometer_enable) {
    node->declare_parameter("magnetometer_publisher.topic_name", "mag");
    _magnetometer_topic =
        node->get_parameter("magnetometer_publisher.topic_name")
            .get_parameter_value()
            .get<std::string>();

    node->declare_parameter("magnetometer_publisher.frame_id", "mag_frame");
    magnetometer_frame_id =
        node->get_parameter("magnetometer_publisher.frame_id")
            .get_parameter_value()
            .get<std::string>();

  load_parameter(magnetometer_enable,"magnetometer_publisher.covariance", 0, magnetometer_covariance);


    load_parameter_f("magnetometer_publisher.coefficient",  1.f, magnetometer_coeff);
    load_parameter_f("magnetometer_publisher.addition",  0.f, magnetometer_addition);


    magnetometer_publisher =
        node->create_publisher<sensor_msgs::msg::MagneticField>(
            _magnetometer_topic, 1);
  }
  /* BAROMETER */
  node->declare_parameter("barometer_publisher.enabled", false);
  barometer_enable = node->get_parameter("barometer_publisher.enabled")
                         .get_parameter_value()
                         .get<bool>();
  if (barometer_enable) {
    node->declare_parameter("barometer_publisher.topic_name", "baro");
    _barometer_topic = node->get_parameter("barometer_publisher.topic_name")
                           .get_parameter_value()
                           .get<std::string>();

    node->declare_parameter("barometer_publisher.frame_id", "baro_frame");
    barometer_frame_id = node->get_parameter("barometer_publisher.frame_id")
                             .get_parameter_value()
                             .get<std::string>();

    load_parameter_d("barometer_publisher.variance",  1.f, barometer_variance);
    load_parameter_d("barometer_publisher.coefficient",  1.f, barometer_coeff);
    load_parameter_d("barometer_publisher.addition",  0.f, barometer_addition);

    barometer_publisher = node->create_publisher<sensor_msgs::msg::FluidPressure>(_barometer_topic, 1);
  }
  /* ALTIMETER */
  node->declare_parameter("altimeter_publisher.enabled", false);
  altimeter_enable = node->get_parameter("altimeter_publisher.enabled")
                         .get_parameter_value()
                         .get<bool>();
  if (barometer_enable) {
    node->declare_parameter("altimeter_publisher.topic_name", "altimeter");
    _altimeter_topic = node->get_parameter("altimeter_publisher.topic_name")
                           .get_parameter_value()
                           .get<std::string>();

  load_parameter_d("altimeter_publisher.coefficient",  1.f, altimeter_coeff);
  load_parameter_d("altimeter_publisher.addition",  0.f, altimeter_addition);

    altimeter_publisher =
        node->create_publisher<std_msgs::msg::Float64>(_altimeter_topic, 1);
  }
  /* ORIENTATION */
  node->declare_parameter("orientation_publisher.enabled", false);
  orientation_enable = node->get_parameter("orientation_publisher.enabled")
                           .get_parameter_value()
                           .get<bool>();

  if (orientation_enable) {
    node->declare_parameter("orientation_publisher.topic_name", "orientation");
    _orientation_topic = node->get_parameter("orientation_publisher.topic_name")
                             .get_parameter_value()
                             .get<std::string>();
    orientation_publisher =
        node->create_publisher<geometry_msgs::msg::Quaternion>(
            _orientation_topic, 1);
  }
  /* GPS */
  node->declare_parameter("gps_publisher.enabled", false);
  gps_enable = node->get_parameter("gps_publisher.enabled")
                   .get_parameter_value()
                   .get<bool>();
  if (gps_enable) {
    node->declare_parameter("gps_publisher.navsat_fix_frame_id", "gps_frame");
    gps_frame_id = node->get_parameter("gps_publisher.navsat_fix_frame_id")
                       .get_parameter_value()
                       .get<std::string>();

    node->declare_parameter("gps_publisher.navsat_fix_topic_name", "gps");
    _gps_topic = node->get_parameter("gps_publisher.navsat_fix_topic_name")
                     .get_parameter_value()
                     .get<std::string>();
    gps_publisher =
        node->create_publisher<sensor_msgs::msg::NavSatFix>(_gps_topic, 1);

    node->declare_parameter("gps_publisher.ground_speed_topic_name", "gps");
    _ground_speed_topic =
        node->get_parameter("gps_publisher.ground_speed_topic_name")
            .get_parameter_value()
            .get<std::string>();

    ground_speed_publisher = node->create_publisher<geometry_msgs::msg::Twist>(
        _ground_speed_topic, 1);


    node->declare_parameter("gps_publisher.navsat_variance_topic_name",
                            "accuracy");
    _accuracy_topic =
        node->get_parameter("gps_publisher.navsat_variance_topic_name")
            .get_parameter_value()
            .get<std::string>();

    accuracy_publisher =
        node->create_publisher<geometry_msgs::msg::Vector3>(_accuracy_topic, 1);


    node->declare_parameter("gps_publisher.navsat_satellites_topic_name",
                            "satellites");
    _satellites_topic =
        node->get_parameter("gps_publisher.navsat_satellites_topic_name")
            .get_parameter_value()
            .get<std::string>();

    satellites_publisher =
        node->create_publisher<std_msgs::msg::UInt32>(_satellites_topic, 1);


    node->declare_parameter("gps_publisher.navsat_altitude_topic_name",
                            "gps_altitude");
    _gps_altitude_topic =
        node->get_parameter("gps_publisher.navsat_altitude_topic_name")
            .get_parameter_value()
            .get<std::string>();

    gps_altitude_publisher =
        node->create_publisher<std_msgs::msg::Float32>(_gps_altitude_topic, 1);
  }
  /* REALTIME CLOCK */
 
  node->declare_parameter("rtc_publisher.enabled", false);
  rtc_enable = node->get_parameter("rtc_publisher.enabled")
                   .get_parameter_value()
                   .get<bool>();
  if (rtc_enable) {
    node->declare_parameter("rtc_publisher.topic_name", "rtc");
    _rtc_topic = node->get_parameter("rtc_publisher.topic_name")
                     .get_parameter_value()
                     .get<std::string>();
 
    rtc_publisher =
        node->create_publisher<rosgraph_msgs::msg::Clock>(_rtc_topic, 1);
   

    node->declare_parameter("rtc_publisher.presync", false);
    rtc_presync = node->get_parameter("rtc_publisher.presync")
                      .get_parameter_value()
                      .get<bool>();
  }


  /*Initializing QT fields*/
  node->declare_parameter("port", "ttyUSB0");
  port_name =
      node->get_parameter("port").get_parameter_value().get<std::string>();

  int int_rate;
  node->declare_parameter("baud_rate", 9600);
  int_rate = node->get_parameter("baud_rate").get_parameter_value().get<int>();

  port_rate = static_cast<QSerialPort::BaudRate>(int_rate);
  reader = new QBaseSerialWitmotionSensorReader(QString(port_name.c_str()),
                                                port_rate);

  int int_interval;
  node->declare_parameter("polling_interval", 10);
  int_interval = node->get_parameter("polling_interval")
                       .get_parameter_value()
                       .get<int>();
  interval = static_cast<uint32_t>(int_interval);
  reader->SetSensorPollInterval(interval);

  int int_timeout_ms;
  node->declare_parameter("timeout_ms", 1000);
  int_timeout_ms = node->get_parameter("timeout_ms")
                       .get_parameter_value()
                       .get<int>();
  timeout_ms = static_cast<uint32_t>(int_timeout_ms);
  reader->SetSensorTimeout(int_timeout_ms);

  reader->ValidatePackets(true);
  reader->moveToThread(&reader_thread);
  connect(&reader_thread, &QThread::finished, reader, &QObject::deleteLater);
  connect(this, &ROSWitmotionSensorController::RunReader, reader,
          &QAbstractWitmotionSensorReader::RunPoll);
  connect(this, &ROSWitmotionSensorController::ConfigureSensor, reader,
          &QAbstractWitmotionSensorReader::SendConfig);
  connect(reader, &QAbstractWitmotionSensorReader::Acquired, this,
          &ROSWitmotionSensorController::Packet);
  connect(reader, &QAbstractWitmotionSensorReader::Error, this,
          &ROSWitmotionSensorController::Error);    

  RCLCPP_INFO(node->get_logger(), "Starting node with lib version (%s).", witmotion::library_version().c_str());   
  reader_thread.start();

}

ROSWitmotionSensorController::~ROSWitmotionSensorController() {
  reader_thread.quit();
  reader_thread.wait(10000);
}

bool ROSWitmotionSensorController::Restart(
    std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response) {
  
  // this is just to prevent compiler from complaining about unused arguments
  (void)request;
  (void)response;

  RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
              "Attempting to restart sensor connection from SUSPENDED state");
  if (!suspended) {
    // ROS_WARN("Cannot restart:  the connection is not suspended");
    RCLCPP_WARN_STREAM_SKIPFIRST(
        rclcpp::get_logger("ROSWitmotionSensorController"),
        "Cannot restart:  the connection is not suspended" << 4);
     return false;
  }
  Instance().Start();
  suspended = false;
  return true;
}

void ROSWitmotionSensorController::imu_process(
    const witmotion_datapacket &packet) {

  if (!(imu_enable_accel || imu_enable_velocities || imu_enable_orientation))
    return;
  static sensor_msgs::msg::Imu msg;
  msg.header.frame_id = imu_frame_id;
  msg.header.stamp = rclcpp::Clock().now();
  for (size_t i = 0; i < 9; i++) {
    msg.linear_acceleration_covariance[i] = imu_accel_covariance[i];
    msg.angular_velocity_covariance[i] = imu_velocity_covariance[i];
    msg.orientation_covariance[i] = imu_orientation_covariance[i];
  }
  static float ax, ay, az, t;
  static float wx, wy, wz;
  static float x, y, z, qx, qy, qz, qw;
  switch (static_cast<witmotion_packet_id>(packet.id_byte)) {
  case pidAcceleration:
    decode_accelerations(packet, ax, ay, az, t);
    imu_have_accel = true;
    break;
  case pidAngularVelocity:
    decode_angular_velocities(packet, wx, wy, wz, t);
    wx *= DEG2RAD;
    wy *= DEG2RAD;
    wz *= DEG2RAD;
    imu_have_velocities = true;
    break;
  case pidAngles:
    decode_angles(packet, x, y, z, t);
    x *= DEG2RAD;
    y *= DEG2RAD;
    z *= DEG2RAD;
    imu_have_orientation = !imu_native_orientation;
    break;
  case pidOrientation:
    decode_orientation(packet, qx, qy, qz, qw);
    imu_have_orientation = imu_native_orientation;
    break;
  default:
    return;
  }
  if (imu_enable_accel && imu_have_accel) {
    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;
  }
  if (imu_enable_velocities && imu_have_velocities) {
    msg.angular_velocity.x = wx;
    msg.angular_velocity.y = wy;
    msg.angular_velocity.z = wz;
  }
  if (imu_enable_orientation && imu_have_orientation) {
    tf2::Quaternion tf_orientation;
    if (imu_native_orientation) {
      tf_orientation.setX(qx);
      tf_orientation.setY(qy);
      tf_orientation.setZ(qz);
      tf_orientation.setW(qw);
    } else
      tf_orientation.setRPY(x, y, z);
    tf_orientation = tf_orientation.normalize();
    msg.orientation = tf2::toMsg(tf_orientation);
  }
  if ((imu_enable_accel == imu_have_accel) &&
      (imu_enable_velocities == imu_have_velocities) &&
      (imu_enable_orientation == imu_have_orientation)) {    
    imu_publisher->publish(msg);
    
    imu_have_accel = false;
    imu_have_velocities = false;
    imu_have_orientation = false;


  }
}

void ROSWitmotionSensorController::temp_process(
    const witmotion_datapacket &packet) {
  if (temp_enable &&
      (static_cast<witmotion_packet_id>(packet.id_byte) == temp_from)) {
    float x, y, z, t;
    static sensor_msgs::msg::Temperature msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = temp_frame_id;
    msg.variance = temp_variance;
    switch (temp_from) {
    case pidAcceleration:
      decode_accelerations(packet, x, y, z, t);
      break;
    case pidAngularVelocity:
      decode_angular_velocities(packet, x, y, z, t);
      break;
    case pidAngles:
      decode_angles(packet, x, y, z, t);
      break;
    case pidMagnetometer:
      decode_magnetometer(packet, x, y, z, t);
      break;
    default:
      return;
    }
    msg.temperature = (t * temp_coeff) + temp_addition;
    temp_publisher->publish(msg);
  }
}

void ROSWitmotionSensorController::magnetometer_process(
    const witmotion_datapacket &packet) {
  if (!magnetometer_enable)
    return;
  static sensor_msgs::msg::MagneticField msg;
  static float x, y, z, t;
  msg.header.frame_id = magnetometer_frame_id;
  msg.header.stamp = rclcpp::Clock().now();
  for (size_t i = 0; i < 9; i++)
    msg.magnetic_field_covariance[i] = magnetometer_covariance[i];
  decode_magnetometer(packet, x, y, z, t);
  msg.magnetic_field.x = (x * magnetometer_coeff) + magnetometer_addition;
  msg.magnetic_field.y = (y * magnetometer_coeff) + magnetometer_addition;
  msg.magnetic_field.z = (z * magnetometer_coeff) + magnetometer_addition;
  magnetometer_publisher->publish(msg);
}

void ROSWitmotionSensorController::altimeter_process(
    const witmotion_datapacket &packet) {
  static double p, h;
  decode_altimeter(packet, p, h);
  if (barometer_enable) {
    static sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = rclcpp::Clock().now();
    msg.header.frame_id = barometer_frame_id;
    msg.variance = barometer_variance;
    msg.fluid_pressure = (p * barometer_coeff) + barometer_addition;
    barometer_publisher->publish(msg);
  }
  if (altimeter_enable) {
    static std_msgs::msg::Float64 msg;
    msg.data = (h * altimeter_coeff) + altimeter_addition;
    if (!have_altitude)
      have_altitude = true;
    last_altitude = msg.data;
    altimeter_publisher->publish(msg);
  }
}

void ROSWitmotionSensorController::orientation_process(
    const witmotion_datapacket &packet) {
  if (orientation_enable) {
    static float x, y, z, w;
    static geometry_msgs::msg::Quaternion msg;
    decode_orientation(packet, x, y, z, w);
    msg.x = x;
    msg.y = y;
    msg.z = z;
    msg.w = w;
    orientation_publisher->publish(msg);
  }
}

void ROSWitmotionSensorController::gps_process(
    const witmotion_datapacket &packet) {
  if (gps_enable) {
    static double latitude_deg, latitude_min, longitude_deg, longitude_min;
    static sensor_msgs::msg::NavSatFix msg;
    decode_gps(packet, longitude_deg, longitude_min, latitude_deg,
               latitude_min);
    msg.header.frame_id = gps_frame_id;
    msg.header.stamp = rclcpp::Clock().now();
    msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    msg.latitude = latitude_deg + (latitude_min / 60.f);
    msg.longitude = longitude_deg + (longitude_min / 60.f);
    msg.altitude = have_ground_speed ? gps_altitude : NAN;
    msg.position_covariance_type =
        have_accuracy
            ? sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN
            : sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    for (uint8_t i = 0; i < 9; i++)
      msg.position_covariance[i] = have_accuracy ? gps_covariance[i] : 0.f;
    gps_publisher->publish(msg);
    have_gps = true;
  }
}

void ROSWitmotionSensorController::ground_speed_process(
    const witmotion_datapacket &packet) {
  if (gps_enable) {
    static float gps_angular_velocity;
    static double gps_ground_speed;
    static std_msgs::msg::Float32 gps_altitude_msg;
    static geometry_msgs::msg::Twist ground_speed_msg;
    decode_gps_ground_speed(packet, gps_altitude, gps_angular_velocity,
                            gps_ground_speed);
    gps_altitude_msg.data = gps_altitude;
    gps_altitude_publisher->publish(gps_altitude_msg);
    ground_speed_msg.linear.x = gps_ground_speed;
    ground_speed_msg.angular.z = gps_angular_velocity;
    ground_speed_publisher->publish(ground_speed_msg);
    have_ground_speed = true;
  }
}

void ROSWitmotionSensorController::accuracy_process(
    const witmotion_datapacket &packet) {
  if (gps_enable) {
    static size_t sat;
    static std_msgs::msg::UInt32 satellites_msg;
    static float local, horizontal, vertical;
    static geometry_msgs::msg::Vector3 accuracy_msg;
    decode_gps_accuracy(packet, sat, local, horizontal, vertical);
    satellites = static_cast<uint32_t>(sat);
    satellites_msg.data = satellites;
    satellites_publisher->publish(satellites_msg);
    accuracy_msg.x = local;
    accuracy_msg.y = horizontal;
    accuracy_msg.z = vertical;
    accuracy_publisher->publish(accuracy_msg);
    gps_covariance[0] = local;
    gps_covariance[4] = horizontal;
    gps_covariance[8] = vertical;
    have_accuracy = true;
  }
}

void ROSWitmotionSensorController::rtc_process(
    const witmotion_datapacket &packet) {
  if (rtc_enable) {
    static uint8_t year, month, day;
    static uint8_t hour, minute, second;
    static uint16_t millisecond;
    static rosgraph_msgs::msg::Clock rtc_msg;
    decode_realtime_clock(packet, year, month, day, hour, minute, second,
                          millisecond);
    QDateTime qt_time;
    qt_time.setTime(QTime(hour, minute, second, millisecond));
    qt_time.setDate(QDate(year + 2000, month, day));
    rtc_msg.clock.sec = qt_time.toSecsSinceEpoch();
    rtc_msg.clock.nanosec = static_cast<uint32_t>(1000000 * millisecond);
    rtc_publisher->publish(rtc_msg);
  }
}

ROSWitmotionSensorController &ROSWitmotionSensorController::Instance() {
  static ROSWitmotionSensorController instance;
  return instance;
}

rclcpp::Node::SharedPtr ROSWitmotionSensorController::Start() {
  RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
              "Controller started");
  emit RunReader();
  //unsigned int microsecond = 1000000;
  //usleep(3 * microsecond);//sleeps for 3 second
  
  if (rtc_enable && rtc_presync) {
    RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
                "Initiating RTC pre-synchonization: current timestamp %s",
                QDateTime::currentDateTime()
                    .toString(Qt::DateFormat::ISODateWithMs)
                    .toStdString()
                    .c_str());
    witmotion::witmotion_config_packet config_packet;
    config_packet.header_byte = witmotion::WITMOTION_CONFIG_HEADER;
    config_packet.key_byte = witmotion::WITMOTION_CONFIG_KEY;
    config_packet.address_byte = ridUnlockConfiguration;
    config_packet.setting.raw[0] = 0x88;
    config_packet.setting.raw[1] = 0xB5;
    RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
                "Configuration ROM: lock removal started");
    emit ConfigureSensor(config_packet);
    sleep(1);
    config_packet.address_byte = witmotion::ridTimeMilliseconds;
    config_packet.setting.bin =
        static_cast<uint16_t>(QDateTime::currentDateTime().time().msec());
    emit ConfigureSensor(config_packet);
    sleep(1);
    config_packet.address_byte = witmotion::ridTimeMinuteSecond;
    config_packet.setting.raw[0] =
        static_cast<uint8_t>(QDateTime::currentDateTime().time().minute());
    config_packet.setting.raw[1] =
        static_cast<uint8_t>(QDateTime::currentDateTime().time().second());
    emit ConfigureSensor(config_packet);
    sleep(1);
    config_packet.address_byte = witmotion::ridTimeDayHour;
    config_packet.setting.raw[0] =
        static_cast<uint8_t>(QDateTime::currentDateTime().date().day());
    config_packet.setting.raw[1] =
        static_cast<uint8_t>(QDateTime::currentDateTime().time().hour());
    emit ConfigureSensor(config_packet);
    sleep(1);
    config_packet.address_byte = witmotion::ridTimeYearMonth;
    config_packet.setting.raw[0] =
        static_cast<int8_t>(QDateTime::currentDateTime().date().year() - 2000);
    config_packet.setting.raw[1] =
        static_cast<uint8_t>(QDateTime::currentDateTime().date().month());
    emit ConfigureSensor(config_packet);
    sleep(1);
    RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
                "RTC pre-synchonization completed, saving configuration");
    config_packet.address_byte = ridSaveSettings;
    config_packet.setting.raw[0] = 0x00;
    config_packet.setting.raw[1] = 0x00;
    emit ConfigureSensor(config_packet);
    sleep(1);
    RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
                "RTC synchronized");
  }

  return node;
}

void ROSWitmotionSensorController::Packet(const witmotion_datapacket &packet) {
  // RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"), "Packet ID
  // 0x%X acquired", packet.id_byte);
  switch (static_cast<witmotion_packet_id>(packet.id_byte)) {
  case pidRTC:
    rtc_process(packet);
    break;
  case pidAcceleration:
  case pidAngularVelocity:
  case pidAngles:
    imu_process(packet);
    temp_process(packet);
    break;
  case pidMagnetometer:
    magnetometer_process(packet);
    temp_process(packet);
    break;
  case pidAltimeter:
    altimeter_process(packet);
    break;
  case pidOrientation:
    orientation_process(packet);
    imu_process(packet);
    break;
  case pidGPSAccuracy:
    accuracy_process(packet);
    break;
  case pidGPSGroundSpeed:
    ground_speed_process(packet);
    break;
  case pidGPSCoordinates:
    gps_process(packet);
    break;
  default:
    RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
                "Unknown packet ID 0x%X acquired", packet.id_byte);
  }
}

void ROSWitmotionSensorController::Error(const QString &description) {
  RCLCPP_ERROR(rclcpp::get_logger("ROSWitmotionSensorController"),
               "Sensor error: %s", description.toStdString().c_str());
  RCLCPP_INFO(rclcpp::get_logger("ROSWitmotionSensorController"),
              "Entering SUSPENDED state");
  reader->Suspend();
  suspended = true;
}

void ROSWitmotionSensorController::load_parameter_d(std::string param_name, double init_val, double &param_var) {
          try{
            node->declare_parameter(param_name, init_val);
            param_var = node->get_parameter(param_name)
                                .get_parameter_value()
                                .get<double>();
          } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {

              RCLCPP_WARN(node->get_logger(), "Exception reading param (%s).\nReading as string vector", ex.what());
              node->declare_parameter( param_name, std::to_string(init_val) );
              auto tmp = node->get_parameter(param_name).get_parameter_value().get<std::string>();  
              param_var = strtod(tmp.c_str(), NULL);
 
          }
}

void ROSWitmotionSensorController::load_parameter_f(std::string param_name, float init_val, float &param_var) {
          try{
            node->declare_parameter(param_name, init_val);
            param_var = node->get_parameter(param_name)
                                .get_parameter_value()
                                .get<float>();
          } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {

              RCLCPP_WARN(node->get_logger(), "Exception reading param (%s).\nReading as string vector", ex.what());
              node->declare_parameter( param_name, std::to_string(init_val) );
              auto tmp = node->get_parameter(param_name).get_parameter_value().get<std::string>();  
              param_var = strtod(tmp.c_str(), NULL);
 
          }
}

void ROSWitmotionSensorController::load_parameter(bool is_active, std::string param_name, double first_val, std::vector<double> &param_vector) {
        if (is_active){
          try{
              node->declare_parameter( param_name, std::vector<double>({first_val, 0, 0, 0, 0, 0, 0, 0, 0}));
              param_vector = node->get_parameter(  param_name).get_parameter_value().get<std::vector<double>>();  
          } catch (const rclcpp::exceptions::InvalidParameterTypeException & ex) {

              RCLCPP_WARN(node->get_logger(), "Exception reading param (%s).\nReading as string vector", ex.what());
              node->declare_parameter( param_name, std::vector<std::string>({std::to_string(first_val), "0", "0", "0", "0", "0", "0", "0", "0"}));
              auto tmp = node->get_parameter(param_name).get_parameter_value().get<std::vector<std::string>>();  

              param_vector.clear();
              for (std::string token : tmp){
                param_vector.push_back(strtod(token.c_str(), NULL));
              }

          }
        }
}
