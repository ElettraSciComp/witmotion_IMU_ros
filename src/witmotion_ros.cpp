#include "witmotion_ros.h"

bool ROSWitmotionSensorController::suspended = false;
ros::ServiceServer* ROSWitmotionSensorController::restart_service = nullptr;

/* IMU */
ros::Publisher* ROSWitmotionSensorController::imu_publisher = nullptr;
std::string ROSWitmotionSensorController::imu_frame_id = "imu";
bool ROSWitmotionSensorController::imu_enable_accel = false;
bool ROSWitmotionSensorController::imu_enable_velocities = false;
bool ROSWitmotionSensorController::imu_enable_orientation = false;
bool ROSWitmotionSensorController::imu_have_accel = false;
bool ROSWitmotionSensorController::imu_have_velocities = false;
bool ROSWitmotionSensorController::imu_have_orientation = false;
bool ROSWitmotionSensorController::imu_native_orientation = false;
std::vector<float> ROSWitmotionSensorController::imu_accel_covariance = {-1,0,0,0,0,0,0,0,0};
std::vector<float> ROSWitmotionSensorController::imu_velocity_covariance = {-1,0,0,0,0,0,0,0,0};
std::vector<float> ROSWitmotionSensorController::imu_orientation_covariance = {-1,0,0,0,0,0,0,0,0};

/* TEMPERATURE */
witmotion_packet_id ROSWitmotionSensorController::temp_from = pidAcceleration;
std::string ROSWitmotionSensorController::temp_frame_id = "imu";
bool ROSWitmotionSensorController::temp_enable = false;
float ROSWitmotionSensorController::temp_variance = 0.f;
float ROSWitmotionSensorController::temp_coeff = 1.f;
float ROSWitmotionSensorController::temp_addition = 0.f;
ros::Publisher* ROSWitmotionSensorController::temp_publisher = nullptr;

/* MAGNETOMETER */
std::string ROSWitmotionSensorController::magnetometer_frame_id = "imu";
bool ROSWitmotionSensorController::magnetometer_enable = false;
std::vector<float> ROSWitmotionSensorController::magnetometer_covariance = {0,0,0,0,0,0,0,0,0};
ros::Publisher* ROSWitmotionSensorController::magnetometer_publisher = nullptr;
float ROSWitmotionSensorController::magnetometer_coeff = 1.f;
float ROSWitmotionSensorController::magnetometer_addition = 0.f;

/* BAROMETER */
std::string ROSWitmotionSensorController::barometer_frame_id = "imu";
bool ROSWitmotionSensorController::barometer_enable = false;
double ROSWitmotionSensorController::barometer_variance = 0.f;
double ROSWitmotionSensorController::barometer_coeff = 1.f;
double ROSWitmotionSensorController::barometer_addition = 0.f;
ros::Publisher* ROSWitmotionSensorController::barometer_publisher = nullptr;

/* ALTIMETER */
bool ROSWitmotionSensorController::altimeter_enable = false;
double ROSWitmotionSensorController::altimeter_coeff = 1.f;
double ROSWitmotionSensorController::altimeter_addition = 0.f;
ros::Publisher* ROSWitmotionSensorController::altimeter_publisher = nullptr;
bool ROSWitmotionSensorController::have_altitude = false;
double ROSWitmotionSensorController::last_altitude = 0.f;

/* ORIENTATION */
bool ROSWitmotionSensorController::orientation_enable = false;
ros::Publisher* ROSWitmotionSensorController::orientation_publisher = nullptr;

/* GPS */
bool ROSWitmotionSensorController::gps_enable = false;
bool ROSWitmotionSensorController::have_gps = false;
bool ROSWitmotionSensorController::have_accuracy = false;
bool ROSWitmotionSensorController::have_ground_speed = false;
uint32_t ROSWitmotionSensorController::satellites = 0;
float ROSWitmotionSensorController::gps_altitude = NAN;
std::vector<double> ROSWitmotionSensorController::gps_covariance = {0,0,0,0,0,0,0,0,0};
std::string ROSWitmotionSensorController::gps_frame_id = "world";
ros::Publisher* ROSWitmotionSensorController::gps_publisher = nullptr;
ros::Publisher* ROSWitmotionSensorController::ground_speed_publisher = nullptr;
ros::Publisher* ROSWitmotionSensorController::accuracy_publisher = nullptr;
ros::Publisher* ROSWitmotionSensorController::satellites_publisher = nullptr;
ros::Publisher* ROSWitmotionSensorController::gps_altitude_publisher = nullptr;

/* REALTIME CLOCK */
bool ROSWitmotionSensorController::rtc_enable = false;
ros::Publisher* ROSWitmotionSensorController::rtc_publisher = nullptr;
bool ROSWitmotionSensorController::rtc_presync = false;

ROSWitmotionSensorController::ROSWitmotionSensorController():
    reader_thread(dynamic_cast<QObject*>(this)),
    node("~")
{
    /*Initializing ROS fields*/
    node.getParam("restart_service_name", _restart_service_name);
    _restart_service = node.advertiseService(_restart_service_name, ROSWitmotionSensorController::Restart);
    restart_service = &_restart_service;
    /* IMU */
    node.param<std::string>("imu_publisher/topic_name", _imu_topic, "imu");
    _imu_publisher = node.advertise<sensor_msgs::Imu>(_imu_topic, 1);
    imu_publisher = &_imu_publisher;
    node.param<std::string>("imu_publisher/frame_id", imu_frame_id, "imu");
    node.param<bool>("imu_publisher/use_native_orientation", imu_native_orientation, false);
    node.param<bool>("imu_publisher/measurements/acceleration/enabled", imu_enable_accel, false);
    if(imu_enable_accel)
        node.getParam("imu_publisher/measurements/acceleration/covariance", imu_accel_covariance);
    node.param<bool>("imu_publisher/measurements/angular_velocity/enabled", imu_enable_velocities, false);
    if(imu_enable_velocities)
        node.getParam("imu_publisher/measurements/angular_velocity/covariance", imu_velocity_covariance);
    node.param<bool>("imu_publisher/measurements/orientation/enabled", imu_enable_orientation, false);
    if(imu_enable_orientation)
        node.getParam("imu_publisher/measurements/orientation/covariance", imu_orientation_covariance);
    /* TEMPERATURE */
    node.param<bool>("temperature_publisher/enabled", temp_enable, false);
    if(temp_enable)
    {
        node.getParam("temperature_publisher/topic_name", _temp_topic);
        node.getParam("temperature_publisher/frame_id", temp_frame_id);
        std::string temp_from_str;
        node.getParam("temperature_publisher/from_message", temp_from_str);
        std::transform(temp_from_str.begin(), temp_from_str.end(), temp_from_str.begin(), ::toupper);
        if(temp_from_str == "ACCELERATION")
            temp_from = pidAcceleration;
        else if(temp_from_str == "ANGULAR_VEL")
            temp_from = pidAngularVelocity;
        else if(temp_from_str == "ORIENTATION")
            temp_from = pidAngles;
        else if(temp_from_str == "MAGNETOMETER")
            temp_from = pidMagnetometer;
        else
        {
            ROS_WARN("Cannot determine message type to take temperature from (%s). Falling back to acceleration message", temp_from_str.c_str());
            temp_from = pidAcceleration;
        }
        node.param<float>("temperature_publisher/variance", temp_variance, 0.f);
        node.param<float>("temperature_publisher/coefficient", temp_coeff, 1.f);
        node.param<float>("temperature_publisher/addition", temp_addition, 0.f);
        _temp_publisher = node.advertise<sensor_msgs::Temperature>(_temp_topic, 1);
        temp_publisher = &_temp_publisher;
    }
    /* MAGNETOMETER */
    node.param<bool>("magnetometer_publisher/enabled", magnetometer_enable, false);
    if(magnetometer_enable)
    {
        node.getParam("magnetometer_publisher/topic_name", _magnetometer_topic);
        node.getParam("magnetometer_publisher/frame_id", magnetometer_frame_id);
        node.getParam("magnetometer_publisher/covariance", magnetometer_covariance);
        node.param<float>("magnetometer_publisher/coefficient", magnetometer_coeff, 1.f);
        node.param<float>("magnetometer_publisher/addition", magnetometer_addition, 0.f);
        _magnetometer_publisher = node.advertise<sensor_msgs::MagneticField>(_magnetometer_topic, 1);
        magnetometer_publisher = &_magnetometer_publisher;
    }
    /* BAROMETER */
    node.param<bool>("barometer_publisher/enabled", barometer_enable, false);
    if(barometer_enable)
    {
        node.getParam("barometer_publisher/topic_name", _barometer_topic);
        node.getParam("barometer_publisher/frame_id", barometer_frame_id);
        node.getParam("barometer_publisher/variance", barometer_variance);
        node.param<double>("barometer_publisher/coefficient", barometer_coeff, 1.f);
        node.param<double>("barometer_publisher/addition", barometer_addition, 0.f);
        _barometer_publisher = node.advertise<sensor_msgs::FluidPressure>(_barometer_topic, 1);
        barometer_publisher = &_barometer_publisher;
    }
    /* ALTIMETER */
    node.param<bool>("altimeter_publisher/enabled", altimeter_enable, false);
    if(barometer_enable)
    {
        node.getParam("altimeter_publisher/topic_name", _altimeter_topic);
        node.param<double>("altimeter_publisher/coefficient", altimeter_coeff, 1.f);
        node.param<double>("altimeter_publisher/addition", altimeter_addition, 0.f);
        _altimeter_publisher = node.advertise<std_msgs::Float64>(_altimeter_topic, 1);
        altimeter_publisher = &_altimeter_publisher;
    }
    /* ORIENTATION */
    node.param<bool>("orientation_publisher/enabled", orientation_enable, false);
    if(orientation_enable)
    {
        node.getParam("orientation_publisher/topic_name", _orientation_topic);
        _orientation_publisher = node.advertise<geometry_msgs::Quaternion>(_orientation_topic, 1);
        orientation_publisher = &_orientation_publisher;
    }
    /* GPS */
    node.param<bool>("gps_publisher/enabled", gps_enable, false);
    if(gps_enable)
    {
        node.getParam("gps_publisher/navsat_fix_frame_id", gps_frame_id);
        node.getParam("gps_publisher/navsat_fix_topic_name", _gps_topic);
        _gps_publisher = node.advertise<sensor_msgs::NavSatFix>(_gps_topic, 1);
        gps_publisher = &_gps_publisher;
        node.getParam("gps_publisher/ground_speed_topic_name", _ground_speed_topic);
        _ground_speed_publisher = node.advertise<geometry_msgs::Twist>(_ground_speed_topic, 1);
        ground_speed_publisher = &_ground_speed_publisher;
        node.getParam("gps_publisher/navsat_variance_topic_name", _accuracy_topic);
        _accuracy_publisher = node.advertise<geometry_msgs::Vector3>(_accuracy_topic, 1);
        accuracy_publisher = &_accuracy_publisher;
        node.getParam("gps_publisher/navsat_satellites_topic_name", _satellites_topic);
        _satellites_publisher = node.advertise<std_msgs::UInt32>(_satellites_topic, 1);
        satellites_publisher = &_satellites_publisher;
        node.getParam("gps_publisher/navsat_altitude_topic_name", _gps_altitude_topic);
        _gps_altitude_publisher = node.advertise<std_msgs::Float32>(_gps_altitude_topic, 1);
        gps_altitude_publisher = &_gps_altitude_publisher;
    }
    /* REALTIME CLOCK */
    node.param<bool>("rtc_publisher/enabled", rtc_enable, false);
    if(rtc_enable)
    {
        node.getParam("rtc_publisher/topic_name", _rtc_topic);
        _rtc_publisher = node.advertise<rosgraph_msgs::Clock>(_rtc_topic, 1);
        rtc_publisher = &_rtc_publisher;
        node.param<bool>("rtc_publisher/presync", rtc_presync, false);
    }

    /*Initializing QT fields*/
    node.param<std::string>("port", port_name, "ttyUSB0");
    int int_rate;
    node.param<int>("baud_rate", int_rate, 9600);
    port_rate = static_cast<QSerialPort::BaudRate>(int_rate);
    reader = new QBaseSerialWitmotionSensorReader(QString(port_name.c_str()), port_rate);
    if(node.hasParam("polling_interval"))
    {
       int int_interval;
       node.param<int>("polling_interval", int_interval, 10);
       interval = static_cast<uint32_t>(int_interval);
       reader->SetSensorPollInterval(interval);
    }
    if(node.hasParam("timeout_ms"))
    {
       int int_timeout_ms;
       node.param<int>("timeout_ms", int_timeout_ms, 10);
       timeout_ms = static_cast<uint32_t>(int_timeout_ms);
       reader->SetSensorTimeout(timeout_ms);
    }
    reader->ValidatePackets(true);
    reader->moveToThread(&reader_thread);
    connect(&reader_thread, &QThread::finished, reader, &QObject::deleteLater);
    connect(this, &ROSWitmotionSensorController::RunReader, reader, &QAbstractWitmotionSensorReader::RunPoll);
    connect(this, &ROSWitmotionSensorController::ConfigureSensor, reader, &QAbstractWitmotionSensorReader::SendConfig);
    connect(reader, &QAbstractWitmotionSensorReader::Acquired, this, &ROSWitmotionSensorController::Packet);
    connect(reader, &QAbstractWitmotionSensorReader::Error, this, &ROSWitmotionSensorController::Error);
    reader_thread.start();
}

ROSWitmotionSensorController::~ROSWitmotionSensorController()
{
    reader_thread.quit();
    reader_thread.wait(10000);
}

bool ROSWitmotionSensorController::Restart(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response)
{
    ROS_INFO("Attempting to restart sensor connection from SUSPENDED state");
    if(!suspended)
    {
        ROS_WARN("Cannot restart:  the connection is not suspended");
        return false;
    }
    Instance().Start();
    suspended = false;
    return true;
}

void ROSWitmotionSensorController::imu_process(const witmotion_datapacket &packet)
{
    if(!(imu_enable_accel || imu_enable_velocities || imu_enable_orientation))
        return;
    static sensor_msgs::Imu msg;
    msg.header.frame_id = imu_frame_id;
    msg.header.stamp = ros::Time::now();
    for(size_t i = 0; i < 9; i++)
    {
        msg.linear_acceleration_covariance[i] = imu_accel_covariance[i];
        msg.angular_velocity_covariance[i] = imu_velocity_covariance[i];
        msg.orientation_covariance[i] = imu_orientation_covariance[i];
    }
    static float ax, ay, az, t;
    static float wx, wy, wz;
    static float x, y, z, qx, qy, qz, qw;
    switch(static_cast<witmotion_packet_id>(packet.id_byte))
    {
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
        decode_angles(packet, x, y ,z, t);
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
    if(imu_enable_accel && imu_have_accel)
    {
        msg.linear_acceleration.x = ax;
        msg.linear_acceleration.y = ay;
        msg.linear_acceleration.z = az;
    }
    if(imu_enable_velocities && imu_have_velocities)
    {
        msg.angular_velocity.x = wx;
        msg.angular_velocity.y = wy;
        msg.angular_velocity.z = wz;
    }
    if(imu_enable_orientation && imu_have_orientation)
    {
        tf2::Quaternion tf_orientation;
        if(imu_native_orientation)
        {
            tf_orientation.setX(qx);
            tf_orientation.setY(qy);
            tf_orientation.setZ(qz);
            tf_orientation.setW(qw);
        }
        else
            tf_orientation.setRPY(x, y, z);
        tf_orientation = tf_orientation.normalize();
        msg.orientation = tf2::toMsg(tf_orientation);
    }
    if( (imu_enable_accel == imu_have_accel) &&
            (imu_enable_velocities == imu_have_velocities) &&
            (imu_enable_orientation == imu_have_orientation) )
    {
        imu_publisher->publish(msg);
        imu_have_accel = false;
        imu_have_velocities = false;
        imu_have_orientation = false;
    }
}

void ROSWitmotionSensorController::temp_process(const witmotion_datapacket &packet)
{
    if(temp_enable && (static_cast<witmotion_packet_id>(packet.id_byte) == temp_from))
    {
        float x, y, z, t;
        static sensor_msgs::Temperature msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = temp_frame_id;
        msg.variance = temp_variance;
        switch(temp_from)
        {
        case pidAcceleration:
            decode_accelerations(packet, x, y, z, t);
            break;
        case pidAngularVelocity:
            decode_angular_velocities(packet, x, y , z, t);
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

void ROSWitmotionSensorController::magnetometer_process(const witmotion_datapacket &packet)
{
    if(!magnetometer_enable)
        return;
    static sensor_msgs::MagneticField msg;
    static float x, y ,z, t;
    msg.header.frame_id = magnetometer_frame_id;
    msg.header.stamp = ros::Time::now();
    for(size_t i = 0; i < 9; i++)
        msg.magnetic_field_covariance[i] = magnetometer_covariance[i];
    decode_magnetometer(packet, x, y, z, t);
    msg.magnetic_field.x = (x * magnetometer_coeff) + magnetometer_addition;
    msg.magnetic_field.y = (y * magnetometer_coeff) + magnetometer_addition;
    msg.magnetic_field.z = (z * magnetometer_coeff) + magnetometer_addition;
    magnetometer_publisher->publish(msg);
}

void ROSWitmotionSensorController::altimeter_process(const witmotion_datapacket &packet)
{
    static double p, h;
    decode_altimeter(packet, p, h);
    if(barometer_enable)
    {
        static sensor_msgs::FluidPressure msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = barometer_frame_id;
        msg.variance = barometer_variance;
        msg.fluid_pressure = (p * barometer_coeff) + barometer_addition;
        barometer_publisher->publish(msg);
    }
    if(altimeter_enable)
    {
        static std_msgs::Float64 msg;
        msg.data = (h * altimeter_coeff) + altimeter_addition;
        if(!have_altitude)
            have_altitude = true;
        last_altitude = msg.data;
        altimeter_publisher->publish(msg);
    }
}

void ROSWitmotionSensorController::orientation_process(const witmotion_datapacket &packet)
{
    if(orientation_enable)
    {
        static float x, y, z, w;
        static geometry_msgs::Quaternion msg;
        decode_orientation(packet, x, y, z, w);
        msg.x = x;
        msg.y = y;
        msg.z = z;
        msg.w = w;
        orientation_publisher->publish(msg);
    }
}

void ROSWitmotionSensorController::gps_process(const witmotion_datapacket &packet)
{
    if(gps_enable)
    {
        static double latitude_deg, latitude_min, longitude_deg, longitude_min;
        static sensor_msgs::NavSatFix msg;
        decode_gps(packet, longitude_deg, longitude_min, latitude_deg, latitude_min);
        msg.header.frame_id = gps_frame_id;
        msg.header.stamp = ros::Time::now();
        msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        msg.latitude = std::floor(latitude_deg) + (latitude_min / 60.f);
        msg.longitude = std::floor(longitude_deg) + (longitude_min / 60.f);
        msg.altitude = have_ground_speed ? gps_altitude : NAN;
        msg.position_covariance_type = have_accuracy ? sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN :
                                                       sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        for(uint8_t i = 0; i < 9; i++)
            msg.position_covariance[i] = have_accuracy ? gps_covariance[i] :
                                                         0.f;
        gps_publisher->publish(msg);
        have_gps = true;
    }
}

void ROSWitmotionSensorController::ground_speed_process(const witmotion_datapacket &packet)
{
    if(gps_enable)
    {
        static float gps_angular_velocity;
        static double gps_ground_speed;
        static std_msgs::Float32 gps_altitude_msg;
        static geometry_msgs::Twist ground_speed_msg;
        decode_gps_ground_speed(packet, gps_altitude, gps_angular_velocity, gps_ground_speed);
        gps_altitude_msg.data = gps_altitude;
        gps_altitude_publisher->publish(gps_altitude_msg);
        ground_speed_msg.linear.x = gps_ground_speed;
        ground_speed_msg.angular.z = gps_angular_velocity;
        ground_speed_publisher->publish(ground_speed_msg);
        have_ground_speed = true;
    }
}

void ROSWitmotionSensorController::accuracy_process(const witmotion_datapacket &packet)
{
    if(gps_enable)
    {
        static size_t sat;
        static std_msgs::UInt32 satellites_msg;
        static float local, horizontal, vertical;
        static geometry_msgs::Vector3 accuracy_msg;
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

void ROSWitmotionSensorController::rtc_process(const witmotion_datapacket &packet)
{
    if(rtc_enable)
    {
        static uint8_t year, month, day;
        static uint8_t hour, minute, second;
        static uint16_t millisecond;
        static rosgraph_msgs::Clock rtc_msg;
        decode_realtime_clock(packet,
                              year,
                              month,
                              day,
                              hour,
                              minute,
                              second,
                              millisecond);
        QDateTime qt_time;
        qt_time.setTime(QTime(hour, minute, second, millisecond));
        qt_time.setDate(QDate(year + 2000, month, day));
        rtc_msg.clock.sec = qt_time.toSecsSinceEpoch();
        rtc_msg.clock.nsec = static_cast<uint32_t>(1000000 * millisecond);
        rtc_publisher->publish(rtc_msg);
    }
}

ROSWitmotionSensorController &ROSWitmotionSensorController::Instance()
{
    static ROSWitmotionSensorController instance;
    return instance;
}

void ROSWitmotionSensorController::Start()
{
    emit RunReader();
    if(rtc_enable && rtc_presync)
    {
        ROS_INFO("Initiating RTC pre-synchonization: current timestamp %s",
                 QDateTime::currentDateTime().toString(Qt::DateFormat::ISODateWithMs).toStdString().c_str());
        witmotion::witmotion_config_packet config_packet;
        config_packet.header_byte = witmotion::WITMOTION_CONFIG_HEADER;
        config_packet.key_byte = witmotion::WITMOTION_CONFIG_KEY;
        config_packet.address_byte = ridUnlockConfiguration;
        config_packet.setting.raw[0] = 0x88;
        config_packet.setting.raw[1] = 0xB5;
        ROS_INFO("Configuration ROM: lock removal started");
        emit ConfigureSensor(config_packet);
        sleep(1);
        config_packet.address_byte = witmotion::ridTimeMilliseconds;
        config_packet.setting.bin = static_cast<uint16_t>(QDateTime::currentDateTime().time().msec());
        emit ConfigureSensor(config_packet);
        sleep(1);
        config_packet.address_byte = witmotion::ridTimeMinuteSecond;
        config_packet.setting.raw[0] = static_cast<uint8_t>(QDateTime::currentDateTime().time().minute());
        config_packet.setting.raw[1] = static_cast<uint8_t>(QDateTime::currentDateTime().time().second());
        emit ConfigureSensor(config_packet);
        sleep(1);
        config_packet.address_byte = witmotion::ridTimeDayHour;
        config_packet.setting.raw[0] = static_cast<uint8_t>(QDateTime::currentDateTime().date().day());
        config_packet.setting.raw[1] = static_cast<uint8_t>(QDateTime::currentDateTime().time().hour());
        emit ConfigureSensor(config_packet);
        sleep(1);
        config_packet.address_byte = witmotion::ridTimeYearMonth;
        config_packet.setting.raw[0] = static_cast<int8_t>(QDateTime::currentDateTime().date().year() - 2000);
        config_packet.setting.raw[1] = static_cast<uint8_t>(QDateTime::currentDateTime().date().month());
        emit ConfigureSensor(config_packet);
        sleep(1);
        ROS_INFO("RTC pre-synchonization completed, saving configuration");
        config_packet.address_byte = ridSaveSettings;
        config_packet.setting.raw[0] = 0x00;
        config_packet.setting.raw[1] = 0x00;
        emit ConfigureSensor(config_packet);
        sleep(1);
        ROS_INFO("RTC synchronized");
    }
}

void ROSWitmotionSensorController::Packet(const witmotion_datapacket &packet)
{
    switch(static_cast<witmotion_packet_id>(packet.id_byte))
    {
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
        ROS_INFO("Unknown packet ID 0x%X acquired", packet.id_byte);
    }
}

void ROSWitmotionSensorController::Error(const QString &description)
{
    ROS_ERROR("Sensor error: %s", description.toStdString().c_str());
    ROS_INFO("Entering SUSPENDED state");
    reader->Suspend();
    suspended = true;
}
