#include "witmotion_ros.h"

/* IMU */
ros::Publisher* ROSWitmotionSensorController::imu_publisher = nullptr;
std::string ROSWitmotionSensorController::imu_frame_id = "imu";
bool ROSWitmotionSensorController::imu_enable_accel = false;
bool ROSWitmotionSensorController::imu_enable_velocities = false;
bool ROSWitmotionSensorController::imu_enable_orientation = false;
bool ROSWitmotionSensorController::imu_have_accel = false;
bool ROSWitmotionSensorController::imu_have_velocities = false;
bool ROSWitmotionSensorController::imu_have_orientation = false;
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

ROSWitmotionSensorController::ROSWitmotionSensorController():
    reader_thread(dynamic_cast<QObject*>(this)),
    node("~")
{
    /*Initializing ROS fields*/
    /* IMU */
    node.param<std::string>("imu_publisher/topic_name", _imu_topic, "imu");
    _imu_publisher = node.advertise<sensor_msgs::Imu>(_imu_topic, 1);
    imu_publisher = &_imu_publisher;
    node.param<std::string>("imu_publisher/frame_id", imu_frame_id, "imu");
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
    reader->ValidatePackets(true);
    reader->moveToThread(&reader_thread);
    connect(&reader_thread, &QThread::finished, reader, &QObject::deleteLater);
    connect(this, &ROSWitmotionSensorController::RunReader, reader, &QAbstractWitmotionSensorReader::RunPoll);
    connect(reader, &QAbstractWitmotionSensorReader::Acquired, this, &ROSWitmotionSensorController::Packet);
    connect(reader, &QAbstractWitmotionSensorReader::Error, this, &ROSWitmotionSensorController::Error);
    reader_thread.start();
}

ROSWitmotionSensorController::~ROSWitmotionSensorController()
{
    reader_thread.quit();
    reader_thread.wait(10000);
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
    static float x, y, z;
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
        imu_have_orientation = true;
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
        tf_orientation.setRPY(x, y, z);
        tf_orientation = tf_orientation.normalize();
        msg.orientation = tf2::toMsg(tf_orientation);
    }
    if( (imu_enable_accel == (imu_enable_accel == imu_have_accel)) &&
            (imu_enable_velocities == (imu_enable_velocities == imu_have_velocities)) &&
            (imu_enable_orientation == (imu_enable_orientation == imu_have_orientation)) )
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
        altimeter_publisher->publish(msg);
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
}

void ROSWitmotionSensorController::Packet(const witmotion_datapacket &packet)
{
    switch(static_cast<witmotion_packet_id>(packet.id_byte))
    {
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
    default:
        ROS_WARN("Unknown packet ID 0x%X acquired", packet.id_byte);
    }
}

void ROSWitmotionSensorController::Error(const QString &description)
{
    ROS_ERROR("Sensor error: %s", description.toStdString().c_str());
    reader->Suspend();
}
