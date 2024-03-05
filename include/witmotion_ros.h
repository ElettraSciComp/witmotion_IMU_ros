#ifndef WITMOTION_ROS
#define WITMOTION_ROS

#include <QFile>
#include <QSerialPort>
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

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

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
    rclcpp::Node::SharedPtr node;
    //std::shared_ptr<rclcpp::Node> node; 
    static bool Restart(std::shared_ptr<std_srvs::srv::Empty::Request> request, std::shared_ptr<std_srvs::srv::Empty::Response> response);
    std::string _restart_service_name;
    
 

    /* IMU */
    std::string _imu_topic;
    static std::string imu_frame_id;
    static bool imu_enable_accel;
    static bool imu_have_accel;
    static std::vector<double> imu_accel_covariance;
    static bool imu_enable_velocities;
    static bool imu_have_velocities;
    static std::vector<double> imu_velocity_covariance;
    static bool imu_enable_orientation;
    static bool imu_have_orientation;
    static bool imu_native_orientation;
    static std::vector<double> imu_orientation_covariance;
    //rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher;
    static void imu_process(const witmotion_datapacket& packet);

    /* TEMPERATURE */
    std::string _temp_topic;
    static witmotion_packet_id temp_from;
    static std::string temp_frame_id;
    static bool temp_enable;
    static float temp_variance;
    static float temp_coeff;
    static float temp_addition;
    //static rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr  temp_publisher;
    static void temp_process(const witmotion_datapacket& packet);

    /* MAGNETOMETER */
    std::string _magnetometer_topic;
    static std::string magnetometer_frame_id;
    static bool magnetometer_enable;
    static std::vector<double> magnetometer_covariance;
    static void magnetometer_process(const witmotion_datapacket& packet);
    static float magnetometer_coeff;
    static float magnetometer_addition;
    //static rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr  magnetometer_publisher;

    /* BAROMETER */
    std::string _barometer_topic;
    static std::string barometer_frame_id;
    static bool barometer_enable;
    static double barometer_variance;
    static double barometer_coeff;
    static double barometer_addition;
    //static rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr   barometer_publisher;

    /* ALTIMETER */
    std::string _altimeter_topic;
    static bool altimeter_enable;
    static bool have_altitude;
    static double last_altitude;
    static double altimeter_coeff;
    static double altimeter_addition;
    //static rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   altimeter_publisher;
    static void altimeter_process(const witmotion_datapacket& packet);

    /* ORIENTATION */
    std::string _orientation_topic;
    static bool orientation_enable;
    //static rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr  orientation_publisher;
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
    //static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr  gps_publisher;
    static void gps_process(const witmotion_datapacket& packet);

    std::string _ground_speed_topic;
    std::string _gps_altitude_topic;
    //static rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr   ground_speed_publisher; 
    //static rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gps_altitude_publisher;
    static void ground_speed_process(const witmotion_datapacket& packet);

    std::string _accuracy_topic;
    //static rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr   accuracy_publisher;  
    std::string _satellites_topic;
    //static rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr satellites_publisher;
    static void accuracy_process(const witmotion_datapacket& packet);

    /* REALTIME CLOCK */
    static bool rtc_enable;
    std::string _rtc_topic;
    //static rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr  rtc_publisher;
    static bool rtc_presync;
    static void rtc_process(const witmotion_datapacket& packet);
public:
    static ROSWitmotionSensorController& Instance();
    rclcpp::Node::SharedPtr Start();
    void load_parameter(bool is_active, std::string param_name, double first_val, std::vector<double> &param_vector);
    void load_parameter_d(std::string param_name, double init_val, double &param_var);
    void load_parameter_f(std::string param_name, float init_val, float &param_var);
public slots:
    void Packet(const witmotion_datapacket& packet);
    void Error(const QString& description);
signals:
    void RunReader();
    void ConfigureSensor(const witmotion_config_packet& config_packet);
};

#endif
