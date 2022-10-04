#include "SparkFunLSM9DS1.h"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>

#define G 9.81
#define M_PI 3.141592653589793238

class LSM9DS1Wrapper: public rclcpp::Node{
    LSM9DS1 m_imu;
    sensor_msgs::msg::Imu m_imuMessage;

    rclcpp::TimerBase::SharedPtr timer_;

    float m_gx, m_gy, m_gz;

    float linear_acceleration_stddev_;

    float angular_velocity_stddev_;

    float magnetic_field_stddev_;
    
    float orientation_stddev_;

    float m_ax, m_ay, m_az;

    float m_mx, m_my, m_mz;

    float m_t;

    bool publish_mag;

    bool publish_accel;

    bool publish_gyro;

    rclcpp::Time accel_time;

    rclcpp::Time gyro_time;

    rclcpp::Time mag_time;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;

    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr temp_pub;

    sensor_msgs::msg::Imu imu_raw_msg;

    sensor_msgs::msg::MagneticField mag_msg;

    std_msgs::msg::Float64 temp_msg;

    std::vector<double> m_imu_orientation;

    public:

    LSM9DS1Wrapper();

    uint8_t assign_address(std::string);

    void imu_thread();

    void process_gyro();

    void process_accel();

    void process_mag();

    void process_temp();

    void publish_msgs();

    void publish_imu_msg();
    
    void publish_mag_msg();

    void publish_temp_msg();

    void calibrate();

};