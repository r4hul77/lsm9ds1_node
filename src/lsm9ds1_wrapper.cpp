#include "lsm9ds1_wrapper.h"

LSM9DS1Wrapper::LSM9DS1Wrapper(): Node("lsm9ds1_node"){
    
    this->declare_parameter<std::string>("dev_path", "/dev/i2c-1");
    this->declare_parameter<std::string>("frame_id", "/imu_link");
    this->declare_parameter<uint8_t>("accelerometer_address", 0x6a);
    this->declare_parameter<uint8_t>("magnetometer_address", 0x1c);

    this->declare_parameter<float>("linear_acceleration_stddev", 1);
    this->declare_parameter<float>("angular_velocity_stddev", 1);
    this->declare_parameter<float>("magnetic_field_stddev", 1);

    std::vector<double> default_orientation = {0, 1, 0};

    this->declare_parameter<std::vector<double>>("default_orientation", default_orientation);

    m_imu.set_node(this);
    
    std::string dev_name = this->get_parameter("dev_path").as_string();

    std::string frame_id = this->get_parameter("frame_id").as_string();



    this->get_parameter(
        "linear_acceleration_stddev", linear_acceleration_stddev_);
    this->get_parameter("angular_velocity_stddev", angular_velocity_stddev_);
    this->get_parameter("magnetic_field_stddev", magnetic_field_stddev_);

    this->get_parameter("default_orientation", m_imu_orientation);

    if(m_imu_orientation.size() != 3){
        RCLCPP_ERROR(this->get_logger(), "Orientation Size miss match required 3 got %d", m_imu_orientation.size());
        m_imu_orientation = default_orientation;
    }

    double total = 0;
    for(auto& d : m_imu_orientation){
        total += d*d;
    }

    if((total - 1) > 0.005){
        RCLCPP_ERROR(this->get_logger(), "Wrong Orientation");    
        m_imu_orientation = default_orientation;
    }



    float linear_acceleration_cov = linear_acceleration_stddev_*linear_acceleration_stddev_;
    
    float angular_velocity_cov = angular_velocity_stddev_*angular_velocity_stddev_;

    float magnetic_field_cov = magnetic_field_stddev_*magnetic_field_stddev_;


    imu_raw_msg.header.frame_id = frame_id;
    imu_raw_msg.linear_acceleration_covariance[0] =
    imu_raw_msg.linear_acceleration_covariance[4] =
    imu_raw_msg.linear_acceleration_covariance[8] =
    linear_acceleration_cov;

    imu_raw_msg.angular_velocity_covariance[0] =
    imu_raw_msg.angular_velocity_covariance[4] =
    imu_raw_msg.angular_velocity_covariance[8] =
    angular_velocity_cov;

    mag_msg.header.frame_id = frame_id;

    mag_msg.magnetic_field_covariance[0] =
    mag_msg.magnetic_field_covariance[4] =
    mag_msg.magnetic_field_covariance[8] = magnetic_field_cov;


    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&LSM9DS1Wrapper::imu_thread, this));
    

    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::QoS(1));

    mag_pub = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", rclcpp::QoS(1));

    temp_pub = this->create_publisher<std_msgs::msg::Float64>("imu/temperature", rclcpp::QoS(1));

    uint8_t accel_addr = assign_address("accelerometer_address");

    uint8_t mag_addr = assign_address("magnetometer_address");

    RCLCPP_INFO(this->get_logger(), "Dev Name %s; Mag Addr %x; Accel Addr %x", dev_name.c_str(), mag_addr, accel_addr);

    uint16_t whoamI = m_imu.begin(dev_name, accel_addr, mag_addr);    

    RCLCPP_INFO(this->get_logger(), "WhoAMI returned %d", whoamI);

}


uint8_t LSM9DS1Wrapper::assign_address(std::string parameter_name){
    int addr = this->get_parameter(parameter_name).as_int();

    if(addr > 0 & addr < 0xFF){
        return addr;
    }
    else{
        RCLCPP_ERROR(this->get_logger(), "%s should be > 0 and < 0xFF but %x was described",
         parameter_name.c_str(), addr);
        
        return 0;
    }
}

void LSM9DS1Wrapper::publish_msgs(){
    RCLCPP_INFO(this->get_logger(), "Publishing Msgs");
    publish_imu_msg();
    publish_mag_msg();
    publish_temp_msg();

}


void LSM9DS1Wrapper::imu_thread(){
    
    RCLCPP_INFO(this->get_logger(), "imu thread called");

    bool readGyro(m_imu.gyroAvailable()), readAccel(m_imu.accelAvailable()),
     readMag(m_imu.magAvailable()), readTemp(m_imu.tempAvailable());

    if(readGyro){
        RCLCPP_INFO(this->get_logger(), "Reading Gyro");
        m_imu.readGyro();
        process_gyro();
    }
    if(readAccel){
        RCLCPP_INFO(this->get_logger(), "Reading Acceleration");
        m_imu.readAccel();
        process_accel();
    }
    if(readMag){
        RCLCPP_INFO(this->get_logger(), "Reading Mag");
        m_imu.readMag();
        process_mag();
    }
    if(readTemp){
        RCLCPP_INFO(this->get_logger(), "Reading Temperature");
        m_imu.readTemp();
        process_temp();
    }
    RCLCPP_INFO(this->get_logger(), "Done Reading, On to publishing !!");
    publish_msgs();

}

void LSM9DS1Wrapper::process_temp(){
    m_t = m_imu.temperature;
    RCLCPP_INFO(this->get_logger(), "[LSM9DS1::process_temp] Temperature : %f", m_t);
}

void LSM9DS1Wrapper::calibrate(){
    m_imu.calibrate(m_imu_orientation);
}

void LSM9DS1Wrapper::process_accel(){

    m_ax = m_imu.calcAccel(m_imu.ax)*G;
    m_ay = m_imu.calcAccel(m_imu.ay)*G;
    m_az = m_imu.calcAccel(m_imu.az)*G;
    accel_time = this->now();
    RCLCPP_INFO(this->get_logger(), "[LSM9DS1Wrapper::process_accel] Ax: %f; Ay: %f; Az: %f", m_ax,
     m_ay, m_az);

    imu_raw_msg.linear_acceleration.x = m_ax;
    imu_raw_msg.linear_acceleration.y = m_ay;
    imu_raw_msg.linear_acceleration.z = m_az;
    publish_accel = true;

}


void LSM9DS1Wrapper::process_mag(){
    
    static double g2t = 1/1000;

    m_mx = m_imu.calcMag(m_imu.mx);
    m_my = m_imu.calcMag(m_imu.my);
    m_mz = m_imu.calcMag(m_imu.mz);
    mag_time = this->now();
    
    RCLCPP_INFO(this->get_logger(), "[LSM9DS1Wrapper::process_mag] Mx: %f; My: %f; Mz: %f", m_mx,
     m_my, m_mz);

    mag_msg.magnetic_field.x = m_mx;
    mag_msg.magnetic_field.y = m_my;
    mag_msg.magnetic_field.z = m_mz;

    mag_msg.header.stamp = mag_time;

    publish_mag = true;
}


void LSM9DS1Wrapper::publish_mag_msg(){
    if(publish_mag){
        auto copy_mag_msg(mag_msg);
        mag_pub->publish(std::move(copy_mag_msg));
    }
    publish_mag = false;
}



void LSM9DS1Wrapper::publish_temp_msg(){
    temp_pub->publish(temp_msg);
}

void LSM9DS1Wrapper::publish_imu_msg(){
    if(publish_accel & publish_gyro){
        auto copy_imu_msg(imu_raw_msg);
        copy_imu_msg.header.stamp = this->now();
        imu_pub->publish(std::move(copy_imu_msg));
        publish_accel = false;
        publish_gyro = false;
    }

}


void LSM9DS1Wrapper::process_gyro(){
    static double d2r = M_PI/180.0;
    m_gx = m_imu.calcGyro(m_imu.gx)*d2r;
    m_gy = m_imu.calcGyro(m_imu.gy)*d2r;
    m_gz = m_imu.calcGyro(m_imu.gz)*d2r;

    gyro_time = this->now();

    RCLCPP_INFO(this->get_logger(), "[LSM9DS1Wrapper::process_gyro] Gx: %f; Gy: %f; Gz: %f", m_gx,
     m_gy, m_gz);

     imu_raw_msg.angular_velocity.x = m_gx;
     imu_raw_msg.angular_velocity.y = m_gy;
     imu_raw_msg.angular_velocity.z = m_gz;
    
    publish_gyro = true;

}




int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LSM9DS1Wrapper>();
    //node->calibrate();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}