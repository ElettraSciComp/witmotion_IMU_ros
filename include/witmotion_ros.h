#ifndef WITMOTION_ROS
#define WITMOTION_ROS

#include <QFile>
#include <QIODevice>
#include <QThread>
#include <QCoreApplication>
#include <QDateTime>

#include "witmotion/types.h"
#include "witmotion/serial.h"

#include <boost/array.hpp>
#include <vector>
#include <algorithm>
#include <boost/range/algorithm.hpp>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/FluidPressure.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt32.h>
#include <std_srvs/Empty.h>
#include <rosgraph_msgs/Clock.h>

using namespace witmotion;

class ROSWitmotionSensorController: public QObject
{
    Q_OBJECT
private:
    /* QT FIELDS */
    std::string port_name;
    QSerialPort::BaudRate port_rate;
    uint32_t interval;
    uint32_t timeout_ms;
    QThread reader_thread;
    QBaseSerialWitmotionSensorReader* reader;
    static bool suspended;

    ROSWitmotionSensorController();
    virtual ~ROSWitmotionSensorController();
    ROSWitmotionSensorController(const ROSWitmotionSensorController& root) = delete;
    ROSWitmotionSensorController operator=(const ROSWitmotionSensorController& root) = delete;

    /* ROS FIELDS*/
    ros::NodeHandle node;
    static bool Restart(std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response);
    std::string _restart_service_name;
    ros::ServiceServer _restart_service;
    static ros::ServiceServer* restart_service;

    /* IMU */
    std::string _imu_topic;
    static std::string imu_frame_id;
    static bool imu_enable_accel;
    static bool imu_have_accel;
    static std::vector<float> imu_accel_covariance;
    static bool imu_enable_velocities;
    static bool imu_have_velocities;
    static std::vector<float> imu_velocity_covariance;
    static bool imu_enable_orientation;
    static bool imu_have_orientation;
    static bool imu_native_orientation;
    static std::vector<float> imu_orientation_covariance;
    ros::Publisher _imu_publisher;
    static ros::Publisher* imu_publisher;
    static void imu_process(const witmotion_datapacket& packet);

    /* TEMPERATURE */
    std::string _temp_topic;
    static witmotion_packet_id temp_from;
    static std::string temp_frame_id;
    static bool temp_enable;
    static float temp_variance;
    static float temp_coeff;
    static float temp_addition;
    ros::Publisher _temp_publisher;
    static ros::Publisher* temp_publisher;
    static void temp_process(const witmotion_datapacket& packet);

    /* MAGNETOMETER */
    std::string _magnetometer_topic;
    static std::string magnetometer_frame_id;
    static bool magnetometer_enable;
    static std::vector<float> magnetometer_covariance;
    static void magnetometer_process(const witmotion_datapacket& packet);
    static float magnetometer_coeff;
    static float magnetometer_addition;
    ros::Publisher _magnetometer_publisher;
    static ros::Publisher* magnetometer_publisher;

    /* BAROMETER */
    std::string _barometer_topic;
    static std::string barometer_frame_id;
    static bool barometer_enable;
    static double barometer_variance;
    static double barometer_coeff;
    static double barometer_addition;
    ros::Publisher _barometer_publisher;
    static ros::Publisher* barometer_publisher;

    /* ALTIMETER */
    std::string _altimeter_topic;
    static bool altimeter_enable;
    static bool have_altitude;
    static double last_altitude;
    static double altimeter_coeff;
    static double altimeter_addition;
    ros::Publisher _altimeter_publisher;
    static ros::Publisher* altimeter_publisher;
    static void altimeter_process(const witmotion_datapacket& packet);

    /* ORIENTATION */
    std::string _orientation_topic;
    static bool orientation_enable;
    ros::Publisher _orientation_publisher;
    static ros::Publisher* orientation_publisher;
    static void orientation_process(const witmotion_datapacket& packet);

    /* GPS */
    static bool gps_enable;
    static bool have_gps;
    static bool have_ground_speed;
    static bool have_accuracy;
    static uint32_t satellites;
    static std::vector<double> gps_covariance;
    static std::string gps_frame_id;
    static float gps_altitude;

    std::string _gps_topic;
    ros::Publisher _gps_publisher;
    static ros::Publisher* gps_publisher;
    static void gps_process(const witmotion_datapacket& packet);

    std::string _ground_speed_topic;
    std::string _gps_altitude_topic;
    ros::Publisher _ground_speed_publisher;
    ros::Publisher _gps_altitude_publisher;
    static ros::Publisher* ground_speed_publisher;
    static ros::Publisher* gps_altitude_publisher;
    static void ground_speed_process(const witmotion_datapacket& packet);

    std::string _accuracy_topic;
    ros::Publisher _accuracy_publisher;
    static ros::Publisher* accuracy_publisher;
    std::string _satellites_topic;
    ros::Publisher _satellites_publisher;
    static ros::Publisher* satellites_publisher;
    static void accuracy_process(const witmotion_datapacket& packet);

    /* REALTIME CLOCK */
    static bool rtc_enable;
    std::string _rtc_topic;
    ros::Publisher _rtc_publisher;
    static ros::Publisher* rtc_publisher;
    static bool rtc_presync;
    static void rtc_process(const witmotion_datapacket& packet);
public:
    static ROSWitmotionSensorController& Instance();
    void Start();
public slots:
    void Packet(const witmotion_datapacket& packet);
    void Error(const QString& description);
signals:
    void RunReader();
    void ConfigureSensor(const witmotion_config_packet& config_packet);
};

#endif
